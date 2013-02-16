/*
	$License:
	Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	$
 */
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/signal.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <linux/poll.h>

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include "mpuirq.h"
#include "slaveirq.h"
#include "mlsl.h"
#include "mldl_cfg.h"
#include <linux/mpu.h>

#if defined (CONFIG_MACH_STAR)
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

struct ext_slave_platform_data_user {
	struct ext_slave_descr *(*get_slave_descr) (void);
	int irq;
	int adapt_num;
	int bus;
	unsigned char address;
	signed char orientation[9];
	void *irq_data;
	void *private_data;
};

struct mpu3050_platform_data {
	__u8 int_config;
	__s8 orientation[GYRO_NUM_AXES * GYRO_NUM_AXES];
	__u8 level_shifter;
	struct ext_slave_platform_data_user accel;
	struct ext_slave_platform_data_user compass;
	struct ext_slave_platform_data_user pressure;
};

struct mldl_cfg_user {
	/* MPU related configuration */
	__u32 requested_sensors;
	__u8  addr;
	__u8  int_config;
	__u8  ext_sync;
	__u8  full_scale;
	__u8  lpf;
	__u8  clk_src;
	__u8  divider;
	__u8  dmp_enable;
	__u8  fifo_enable;
	__u8  dmp_cfg1;
	__u8  dmp_cfg2;
	__u8  gyro_power;
	__u8  offset_tc[GYRO_NUM_AXES];
	__u16 offset[GYRO_NUM_AXES];
	__u8  ram[MPU_MEM_NUM_RAM_BANKS][MPU_MEM_BANK_SIZE];

	/* MPU Related stored status and info */
	__u8  silicon_revision;
	__u8  product_id;
	__u16 trim;

	/* Driver/Kernel related state information */
	int gyro_is_bypassed;
	int dmp_is_running;
	int gyro_is_suspended;
	int accel_is_suspended;
	int compass_is_suspended;
	int pressure_is_suspended;
	int gyro_needs_reset;

	/* Slave related information */
	struct ext_slave_descr *accel;
	struct ext_slave_descr *compass;
	struct ext_slave_descr *pressure;

	/* Platform Data */
	struct mpu3050_platform_data *pdata;
};
#endif

/* Platform data for the MPU */
struct mpu_private_data {
	struct miscdevice dev;
	struct i2c_client *client;

	/* mldl_cfg data */
	struct mldl_cfg mldl_cfg;
	struct mpu_ram		mpu_ram;
	struct mpu_gyro_cfg	mpu_gyro_cfg;
	struct mpu_offsets	mpu_offsets;
	struct mpu_chip_info	mpu_chip_info;
	struct inv_mpu_cfg	inv_mpu_cfg;
	struct inv_mpu_state	inv_mpu_state;

	struct mutex mutex;
	wait_queue_head_t mpu_event_wait;
	struct completion completion;
	struct timer_list timeout;
	struct notifier_block nb;
	struct mpuirq_data mpu_pm_event;
	int response_timeout;	/* In seconds */
	unsigned long event;
	int pid;
	struct module *slave_modules[EXT_SLAVE_NUM_TYPES];

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

struct mpu_private_data *mpu_private_data;

static void mpu_pm_timeout(u_long data)
{
	struct mpu_private_data *mpu = (struct mpu_private_data *)data;
	struct i2c_client *client = mpu->client;
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	complete(&mpu->completion);
}

static int mpu_pm_notifier_callback(struct notifier_block *nb,
				    unsigned long event, void *unused)
{
	struct mpu_private_data *mpu =
	    container_of(nb, struct mpu_private_data, nb);
	struct i2c_client *client = mpu->client;
	struct timeval event_time;
	dev_dbg(&client->adapter->dev, "%s: %ld\n", __func__, event);

	/* Prevent the file handle from being closed before we initialize
	   the completion event */
	mutex_lock(&mpu->mutex);
	if (!(mpu->pid) ||
	    (event != PM_SUSPEND_PREPARE && event != PM_POST_SUSPEND)) {
		mutex_unlock(&mpu->mutex);
		return NOTIFY_OK;
	}

	if (event == PM_SUSPEND_PREPARE)
		mpu->event = MPU_PM_EVENT_SUSPEND_PREPARE;
	if (event == PM_POST_SUSPEND)
		mpu->event = MPU_PM_EVENT_POST_SUSPEND;

	do_gettimeofday(&event_time);
	mpu->mpu_pm_event.interruptcount++;
	mpu->mpu_pm_event.irqtime =
	    (((long long)event_time.tv_sec) << 32) + event_time.tv_usec;
	mpu->mpu_pm_event.data_type = MPUIRQ_DATA_TYPE_PM_EVENT;
	mpu->mpu_pm_event.data = mpu->event;

	if (mpu->response_timeout > 0) {
		mpu->timeout.expires = jiffies + mpu->response_timeout * HZ;
		add_timer(&mpu->timeout);
	}
	INIT_COMPLETION(mpu->completion);
	mutex_unlock(&mpu->mutex);

	wake_up_interruptible(&mpu->mpu_event_wait);
	wait_for_completion(&mpu->completion);
	del_timer_sync(&mpu->timeout);
	dev_dbg(&client->adapter->dev, "%s: %ld DONE\n", __func__, event);
	return NOTIFY_OK;
}

static int mpu_dev_open(struct inode *inode, struct file *file)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	struct i2c_client *client = mpu->client;
	int result;
	int ii;
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	dev_dbg(&client->adapter->dev, "current->pid %d\n", current->pid);

	result = mutex_lock_interruptible(&mpu->mutex);
	if (mpu->pid) {
		mutex_unlock(&mpu->mutex);
		return -EBUSY;
	}
	mpu->pid = current->pid;

	/* Reset the sensors to the default */
	if (result) {
		dev_err(&client->adapter->dev,
			"%s: mutex_lock_interruptible returned %d\n",
			__func__, result);
		return result;
	}

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++)
		__module_get(mpu->slave_modules[ii]);

	mutex_unlock(&mpu->mutex);
	return 0;
}

/* close function - called when the "file" /dev/mpu is closed in userspace   */
static int mpu_release(struct inode *inode, struct file *file)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	struct i2c_client *client = mpu->client;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	int result = 0;
	int ii;
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;

	mutex_lock(&mpu->mutex);
	mldl_cfg->inv_mpu_cfg->requested_sensors = 0;
	result = inv_mpu_suspend(mldl_cfg,
				slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
				slave_adapter[EXT_SLAVE_TYPE_ACCEL],
				slave_adapter[EXT_SLAVE_TYPE_COMPASS],
				slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
				INV_ALL_SENSORS);
	mpu->pid = 0;
	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++)
		module_put(mpu->slave_modules[ii]);

	mutex_unlock(&mpu->mutex);
	complete(&mpu->completion);
	dev_dbg(&client->adapter->dev, "mpu_release\n");

	return result;
}

#if defined (CONFIG_MACH_STAR)
static noinline int mpu_dev_ioctl_rdrw(struct i2c_client *client,
		unsigned long arg)
{
	struct i2c_rdwr_ioctl_data rdwr_arg;
	struct i2c_msg *rdwr_pa;
	u8 __user **data_ptrs;
	int i, res;

	if (copy_from_user(&rdwr_arg,
			   (struct i2c_rdwr_ioctl_data __user *)arg,
			   sizeof(rdwr_arg)))
		return -EFAULT;

	/* Put an arbitrary limit on the number of messages that can
	 * be sent at once */
	if (rdwr_arg.nmsgs > I2C_RDRW_IOCTL_MAX_MSGS)
		return -EINVAL;

	rdwr_pa = kmalloc(rdwr_arg.nmsgs * sizeof(struct i2c_msg), GFP_KERNEL);
	if (!rdwr_pa)
		return -ENOMEM;

	if (copy_from_user(rdwr_pa, rdwr_arg.msgs,
			   rdwr_arg.nmsgs * sizeof(struct i2c_msg))) {
		kfree(rdwr_pa);
		return -EFAULT;
	}

	data_ptrs = kmalloc(rdwr_arg.nmsgs * sizeof(u8 __user *), GFP_KERNEL);
	if (data_ptrs == NULL) {
		kfree(rdwr_pa);
		return -ENOMEM;
	}

	res = 0;
	for (i = 0; i < rdwr_arg.nmsgs; i++) {
		/* Limit the size of the message to a sane amount;
		 * and don't let length change either. */
		if ((rdwr_pa[i].len > 8192) ||
		    (rdwr_pa[i].flags & I2C_M_RECV_LEN)) {
			res = -EINVAL;
			break;
		}
		data_ptrs[i] = (u8 __user *)rdwr_pa[i].buf;
		rdwr_pa[i].buf = memdup_user(data_ptrs[i], rdwr_pa[i].len);
		if (IS_ERR(rdwr_pa[i].buf)) {
			res = PTR_ERR(rdwr_pa[i].buf);
			break;
		}
	}
	if (res < 0) {
		int j;
		for (j = 0; j < i; ++j)
			kfree(rdwr_pa[j].buf);
		kfree(data_ptrs);
		kfree(rdwr_pa);
		return res;
	}

	res = i2c_transfer(client->adapter, rdwr_pa, rdwr_arg.nmsgs);
	while (i-- > 0) {
		if (res >= 0 && (rdwr_pa[i].flags & I2C_M_RD)) {
			if (copy_to_user(data_ptrs[i], rdwr_pa[i].buf,
					 rdwr_pa[i].len))
				res = -EFAULT;
		}
		kfree(rdwr_pa[i].buf);
	}
	kfree(data_ptrs);
	kfree(rdwr_pa);
	return res;
}
#endif

/* read function called when from /dev/mpu is read.  Read from the FIFO */
static ssize_t mpu_read(struct file *file,
			char __user *buf, size_t count, loff_t *offset)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	struct i2c_client *client = mpu->client;
	size_t len = sizeof(mpu->mpu_pm_event) + sizeof(unsigned long);
	int err;

	if (!mpu->event && (!(file->f_flags & O_NONBLOCK)))
		wait_event_interruptible(mpu->mpu_event_wait, mpu->event);

	if (!mpu->event || !buf
	    || count < sizeof(mpu->mpu_pm_event))
		return 0;

	err = copy_to_user(buf, &mpu->mpu_pm_event, sizeof(mpu->mpu_pm_event));
	if (err) {
		dev_err(&client->adapter->dev,
			"Copy to user returned %d\n", err);
		return -EFAULT;
	}
	mpu->event = 0;
	return len;
}

static unsigned int mpu_poll(struct file *file, struct poll_table_struct *poll)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	int mask = 0;

	poll_wait(file, &mpu->mpu_event_wait, poll);
	if (mpu->event)
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

static int mpu_dev_ioctl_get_ext_slave_platform_data(
	struct i2c_client *client,
	struct ext_slave_platform_data __user *arg)
{
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct ext_slave_platform_data *pdata_slave;
	struct ext_slave_platform_data local_pdata_slave;

	if (copy_from_user(&local_pdata_slave, arg, sizeof(local_pdata_slave)))
		return -EFAULT;

	if (local_pdata_slave.type >= EXT_SLAVE_NUM_TYPES)
		return -EINVAL;

	pdata_slave = mpu->mldl_cfg.pdata_slave[local_pdata_slave.type];
	/* All but private data and irq_data */
	if (!pdata_slave)
		return -ENODEV;
	if (copy_to_user(arg, pdata_slave, sizeof(*pdata_slave)))
		return -EFAULT;
	return 0;
}

static int mpu_dev_ioctl_get_mpu_platform_data(
	struct i2c_client *client,
	struct mpu_platform_data __user *arg)
{
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mpu_platform_data *pdata = mpu->mldl_cfg.pdata;

	if (copy_to_user(arg, pdata, sizeof(*pdata)))
		return -EFAULT;
	return 0;
}

static int mpu_dev_ioctl_get_ext_slave_descr(
	struct i2c_client *client,
	struct ext_slave_descr __user *arg)
{
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct ext_slave_descr *slave;
	struct ext_slave_descr local_slave;

	if (copy_from_user(&local_slave, arg, sizeof(local_slave)))
		return -EFAULT;

	if (local_slave.type >= EXT_SLAVE_NUM_TYPES)
		return -EINVAL;

	slave = mpu->mldl_cfg.slave[local_slave.type];
	/* All but private data and irq_data */
	if (!slave)
		return -ENODEV;
	if (copy_to_user(arg, slave, sizeof(*slave)))
		return -EFAULT;
	return 0;
}


/**
 * slave_config() - Pass a requested slave configuration to the slave sensor
 *
 * @adapter the adaptor to use to communicate with the slave
 * @mldl_cfg the mldl configuration structuer
 * @slave pointer to the slave descriptor
 * @usr_config The configuration to pass to the slave sensor
 *
 * returns 0 or non-zero error code
 */
static int inv_mpu_config(struct mldl_cfg *mldl_cfg,
			void *gyro_adapter,
			struct ext_slave_config __user *usr_config)
{
	int retval = 0;
#if defined (CONFIG_MACH_STAR)
	int ii;
	struct mldl_cfg_user *temp_mldl_cfg;

	temp_mldl_cfg = kzalloc(sizeof(struct mldl_cfg_user), GFP_KERNEL);
	if (NULL == temp_mldl_cfg)
		return -ENOMEM;

	/*
	 * User space is not allowed to modify accel compass pressure or
	 * pdata structs, as well as silicon_revision product_id or trim
	 */
	if (copy_from_user(temp_mldl_cfg, (struct mldl_cfg_user __user *) usr_config,
				offsetof(struct mldl_cfg_user, silicon_revision))) {
		retval = -EFAULT;
		goto out;
	}

	if (mldl_cfg->inv_mpu_state->status & MPU_GYRO_IS_SUSPENDED)
	{
		if (mldl_cfg->mpu_chip_info->addr != temp_mldl_cfg->addr)
			mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		if (mldl_cfg->mpu_gyro_cfg->int_config != temp_mldl_cfg->int_config)
			mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		if (mldl_cfg->mpu_gyro_cfg->ext_sync != temp_mldl_cfg->ext_sync)
			mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		if (mldl_cfg->mpu_gyro_cfg->full_scale != temp_mldl_cfg->full_scale)
			mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		if (mldl_cfg->mpu_gyro_cfg->lpf != temp_mldl_cfg->lpf)
			mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		if (mldl_cfg->mpu_gyro_cfg->clk_src != temp_mldl_cfg->clk_src)
			mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		if (mldl_cfg->mpu_gyro_cfg->divider != temp_mldl_cfg->divider)
			mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		if (mldl_cfg->mpu_gyro_cfg->dmp_enable != temp_mldl_cfg->dmp_enable)
			mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		if (mldl_cfg->mpu_gyro_cfg->fifo_enable != temp_mldl_cfg->fifo_enable)
			mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		if (mldl_cfg->mpu_gyro_cfg->dmp_cfg1 != temp_mldl_cfg->dmp_cfg1)
			mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		if (mldl_cfg->mpu_gyro_cfg->dmp_cfg2 != temp_mldl_cfg->dmp_cfg2)
			mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		for (ii = 0; ii < GYRO_NUM_AXES; ii++)
			if (mldl_cfg->mpu_offsets->tc[ii] !=
					temp_mldl_cfg->offset_tc[ii])
				mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		for (ii = 0; ii < GYRO_NUM_AXES; ii++)
			if (mldl_cfg->mpu_offsets->gyro[ii] != temp_mldl_cfg->offset[ii])
				mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;

		if (memcmp(mldl_cfg->mpu_ram->ram, temp_mldl_cfg->ram, 
					mldl_cfg->mpu_ram->length))
			mldl_cfg->inv_mpu_state->status |= MPU_GYRO_NEEDS_CONFIG;
	}

	mldl_cfg->inv_mpu_cfg->requested_sensors	= temp_mldl_cfg->requested_sensors;
	mldl_cfg->mpu_chip_info->addr 			= temp_mldl_cfg->addr;
	mldl_cfg->mpu_gyro_cfg->int_config 		= temp_mldl_cfg->int_config;
	mldl_cfg->mpu_gyro_cfg->ext_sync	 	= temp_mldl_cfg->ext_sync;
	mldl_cfg->mpu_gyro_cfg->full_scale 		= temp_mldl_cfg->full_scale;
	mldl_cfg->mpu_gyro_cfg->lpf 			= temp_mldl_cfg->lpf;
	mldl_cfg->mpu_gyro_cfg->clk_src 		= temp_mldl_cfg->clk_src;
	mldl_cfg->mpu_gyro_cfg->divider 		= temp_mldl_cfg->divider;
	mldl_cfg->mpu_gyro_cfg->dmp_enable 		= temp_mldl_cfg->dmp_enable;
	mldl_cfg->mpu_gyro_cfg->fifo_enable 		= temp_mldl_cfg->fifo_enable;
	mldl_cfg->mpu_gyro_cfg->dmp_cfg1 		= temp_mldl_cfg->dmp_cfg1;
	mldl_cfg->mpu_gyro_cfg->dmp_cfg2 		= temp_mldl_cfg->dmp_cfg2;
	memcpy(mldl_cfg->mpu_offsets->tc, temp_mldl_cfg->offset_tc, GYRO_NUM_AXES * sizeof(__u8));
	memcpy(mldl_cfg->mpu_offsets->gyro, temp_mldl_cfg->offset, GYRO_NUM_AXES * sizeof(__u16));
	memcpy(mldl_cfg->mpu_ram->ram, temp_mldl_cfg->ram, mldl_cfg->mpu_ram->length);

out:
	kfree(temp_mldl_cfg);
#else
	struct ext_slave_config config;

	retval = copy_from_user(&config, usr_config, sizeof(config));
	if (retval)
		return -EFAULT;

	if (config.len && config.data) {
		void *data;
		data = kmalloc(config.len, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)config.data, config.len);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		config.data = data;
	}
	retval = gyro_config(gyro_adapter, mldl_cfg, &config);
	kfree(config.data);
#endif
	return retval;
}

static int inv_mpu_get_config(struct mldl_cfg *mldl_cfg,
			    void *gyro_adapter,
			    struct ext_slave_config __user *usr_config)
{
	int retval = 0;
#if defined (CONFIG_MACH_STAR)
	struct mldl_cfg_user *local_mldl_cfg;
	struct mpu3050_platform_data *local_pdata;

	local_mldl_cfg = kzalloc(sizeof(struct mldl_cfg_user), GFP_KERNEL);
	if (NULL == local_mldl_cfg)
		return -ENOMEM;

	local_pdata = kzalloc(sizeof(struct mpu3050_platform_data), GFP_KERNEL);
	if (NULL == local_pdata)
	{
		kfree(local_mldl_cfg);
		return -ENOMEM;
	}

	retval =
	    copy_from_user(local_mldl_cfg, (struct mldl_cfg_user __user *) usr_config,
			   sizeof(struct mldl_cfg_user));
	if (retval) {
		retval = -EFAULT;
		goto out;
	}

	/* Fill in the accel, compass, pressure and pdata pointers */
	if (mldl_cfg->slave[EXT_SLAVE_TYPE_ACCEL]) {
		retval = copy_to_user((void __user *)local_mldl_cfg->accel,
				      mldl_cfg->slave[EXT_SLAVE_TYPE_ACCEL],
				      sizeof(*mldl_cfg->slave[EXT_SLAVE_TYPE_ACCEL]));
		if (retval) {
			retval = -EFAULT;
			goto out;
		}
	}

	if (mldl_cfg->slave[EXT_SLAVE_TYPE_COMPASS]) {
		retval = copy_to_user((void __user *)local_mldl_cfg->compass,
				      mldl_cfg->slave[EXT_SLAVE_TYPE_COMPASS],
				      sizeof(*mldl_cfg->slave[EXT_SLAVE_TYPE_COMPASS]));
		if (retval) {
			retval = -EFAULT;
			goto out;
		}
	}

	if (mldl_cfg->slave[EXT_SLAVE_TYPE_PRESSURE]) {
		retval = copy_to_user((void __user *)local_mldl_cfg->pressure,
				      mldl_cfg->slave[EXT_SLAVE_TYPE_PRESSURE],
				      sizeof(*mldl_cfg->slave[EXT_SLAVE_TYPE_PRESSURE]));
		if (retval) {
			retval = -EFAULT;
			goto out;
		}
	}

	if (mldl_cfg->pdata) {
		local_pdata->int_config = mldl_cfg->pdata->int_config;
		memcpy(local_pdata->orientation, mldl_cfg->pdata->orientation,
				GYRO_NUM_AXES * GYRO_NUM_AXES * sizeof(__s8));
		local_pdata->level_shifter = mldl_cfg->pdata->level_shifter;

		if (mldl_cfg->slave[EXT_SLAVE_TYPE_ACCEL])
		{
			local_pdata->accel.get_slave_descr = mldl_cfg->slave[EXT_SLAVE_TYPE_ACCEL];
			local_pdata->accel.irq = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_ACCEL]->irq;
			local_pdata->accel.adapt_num = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_ACCEL]->adapt_num;
			local_pdata->accel.bus = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_ACCEL]->bus;
			local_pdata->accel.address = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_ACCEL]->address;
			memcpy(local_pdata->accel.orientation, mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_ACCEL]->orientation, 9 * sizeof(__s8));
			local_pdata->accel.irq_data = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_ACCEL]->irq_data;
			local_pdata->accel.private_data = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_ACCEL]->private_data;;
		}

		if (mldl_cfg->slave[EXT_SLAVE_TYPE_COMPASS])
		{
			local_pdata->compass.get_slave_descr = mldl_cfg->slave[EXT_SLAVE_TYPE_COMPASS];
			local_pdata->compass.irq = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_COMPASS]->irq;
			local_pdata->compass.adapt_num = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_COMPASS]->adapt_num;
			local_pdata->compass.bus = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_COMPASS]->bus;
			local_pdata->compass.address = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_COMPASS]->address;
			memcpy(local_pdata->compass.orientation, mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_COMPASS]->orientation, 9 * sizeof(__s8));
			local_pdata->compass.irq_data = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_COMPASS]->irq_data;
			local_pdata->compass.private_data = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_COMPASS]->private_data;
		}

		if (mldl_cfg->slave[EXT_SLAVE_TYPE_PRESSURE])
		{
			local_pdata->pressure.get_slave_descr = mldl_cfg->slave[EXT_SLAVE_TYPE_PRESSURE];
			local_pdata->pressure.irq = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_PRESSURE]->irq;
			local_pdata->pressure.adapt_num = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_PRESSURE]->adapt_num;
			local_pdata->pressure.bus = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_PRESSURE]->bus;
			local_pdata->pressure.address = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_PRESSURE]->address;
			memcpy(local_pdata->pressure.orientation, mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_PRESSURE]->orientation, 9 * sizeof(__s8));
			local_pdata->pressure.irq_data = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_PRESSURE]->irq_data;
			local_pdata->pressure.private_data = mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_PRESSURE]->private_data;
		}

		retval = copy_to_user((void __user *)local_mldl_cfg->pdata,
				      local_pdata,
				      sizeof(*local_pdata));
		if (retval) {
			retval = -EFAULT;
			goto out;
		}
	}

	local_mldl_cfg->requested_sensors	= mldl_cfg->inv_mpu_cfg->requested_sensors;
	local_mldl_cfg->addr 			= mldl_cfg->mpu_chip_info->addr;
	local_mldl_cfg->int_config 		= mldl_cfg->mpu_gyro_cfg->int_config;
	local_mldl_cfg->ext_sync 		= mldl_cfg->mpu_gyro_cfg->ext_sync;
	local_mldl_cfg->full_scale 		= mldl_cfg->mpu_gyro_cfg->full_scale;
	local_mldl_cfg->lpf 			= mldl_cfg->mpu_gyro_cfg->lpf;
	local_mldl_cfg->clk_src 		= mldl_cfg->mpu_gyro_cfg->clk_src;
	local_mldl_cfg->divider 		= mldl_cfg->mpu_gyro_cfg->divider;
	local_mldl_cfg->dmp_enable 		= mldl_cfg->mpu_gyro_cfg->dmp_enable;
	local_mldl_cfg->fifo_enable 		= mldl_cfg->mpu_gyro_cfg->fifo_enable;
	local_mldl_cfg->dmp_cfg1 		= mldl_cfg->mpu_gyro_cfg->dmp_cfg1;
	local_mldl_cfg->dmp_cfg2 		= mldl_cfg->mpu_gyro_cfg->dmp_cfg2;
	memcpy(local_mldl_cfg->offset_tc, mldl_cfg->mpu_offsets->tc, GYRO_NUM_AXES * sizeof(__u8));
	memcpy(local_mldl_cfg->offset, mldl_cfg->mpu_offsets->gyro, GYRO_NUM_AXES * sizeof(__u16));
	memcpy(local_mldl_cfg->ram, mldl_cfg->mpu_ram->ram, mldl_cfg->mpu_ram->length);
	local_mldl_cfg->silicon_revision 	= mldl_cfg->mpu_chip_info->silicon_revision;
	local_mldl_cfg->product_id	 	=  mldl_cfg->mpu_chip_info->product_id;
	local_mldl_cfg->trim		 	= mldl_cfg->mpu_chip_info->gyro_sens_trim;
	local_mldl_cfg->gyro_is_bypassed 	= (mldl_cfg->inv_mpu_state->status & MPU_GYRO_IS_BYPASSED) ? 1 : 0;
	local_mldl_cfg->dmp_is_running	 	= (mldl_cfg->inv_mpu_state->status & MPU_DMP_IS_SUSPENDED) ? 0 : 1;
	local_mldl_cfg->gyro_is_suspended 	= (mldl_cfg->inv_mpu_state->status & MPU_GYRO_IS_SUSPENDED) ? 1 : 0;
	local_mldl_cfg->accel_is_suspended 	= (mldl_cfg->inv_mpu_state->status & MPU_ACCEL_IS_SUSPENDED) ? 1 : 0;
	local_mldl_cfg->compass_is_suspended 	= (mldl_cfg->inv_mpu_state->status & MPU_COMPASS_IS_SUSPENDED) ? 1 : 0;
	local_mldl_cfg->pressure_is_suspended 	= (mldl_cfg->inv_mpu_state->status & MPU_PRESSURE_IS_SUSPENDED) ? 1 : 0;
	local_mldl_cfg->gyro_needs_reset 	= (mldl_cfg->inv_mpu_state->status & MPU_GYRO_NEEDS_CONFIG) ? 1 : 0;

	/* Do not modify the accel, compass, pressure and pdata pointers */
	retval = copy_to_user((struct mldl_cfg_user __user *) usr_config,
			      local_mldl_cfg, offsetof(struct mldl_cfg_user, accel));

	if (retval)
		retval = -EFAULT;
out:
	kfree(local_mldl_cfg);
	kfree(local_pdata);
#else
	struct ext_slave_config config;
	void *user_data;

	retval = copy_from_user(&config, usr_config, sizeof(config));
	if (retval)
		return -EFAULT;

	user_data = config.data;
	if (config.len && config.data) {
		void *data;
		data = kmalloc(config.len, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)config.data, config.len);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		config.data = data;
	}
	retval = gyro_get_config(gyro_adapter, mldl_cfg, &config);
	if (!retval)
		retval = copy_to_user((unsigned char __user *)user_data,
				config.data, config.len);
	kfree(config.data);
#endif
	return retval;
}

static int slave_config(struct mldl_cfg *mldl_cfg,
			void *gyro_adapter,
			void *slave_adapter,
			struct ext_slave_descr *slave,
			struct ext_slave_platform_data *pdata,
			struct ext_slave_config __user *usr_config)
{
	int retval = 0;
	struct ext_slave_config config;
	if ((!slave) || (!slave->config))
#if defined (CONFIG_MACH_STAR)
		return retval;
#else
		return -ENODEV;
#endif

	retval = copy_from_user(&config, usr_config, sizeof(config));
	if (retval)
		return -EFAULT;

	if (config.len && config.data) {
		void *data;
		data = kmalloc(config.len, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)config.data, config.len);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		config.data = data;
	}
	retval = inv_mpu_slave_config(mldl_cfg, gyro_adapter, slave_adapter,
				      &config, slave, pdata);
	kfree(config.data);
	return retval;
}

static int slave_get_config(struct mldl_cfg *mldl_cfg,
			    void *gyro_adapter,
			    void *slave_adapter,
			    struct ext_slave_descr *slave,
			    struct ext_slave_platform_data *pdata,
			    struct ext_slave_config __user *usr_config)
{
	int retval = 0;
	struct ext_slave_config config;
	void *user_data;
#ifndef CONFIG_MACH_STAR
	if (!(slave) || !(slave->get_config))
		return -ENODEV;
#else
		return retval;
#endif

	retval = copy_from_user(&config, usr_config, sizeof(config));
	if (retval)
		return -EFAULT;

	user_data = config.data;
	if (config.len && config.data) {
		void *data;
		data = kmalloc(config.len, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)config.data, config.len);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		config.data = data;
	}
	retval = inv_mpu_get_slave_config(mldl_cfg, gyro_adapter,
					  slave_adapter, &config, slave, pdata);
	if (retval) {
		kfree(config.data);
		return retval;
	}
	retval = copy_to_user((unsigned char __user *)user_data,
			      config.data, config.len);
	kfree(config.data);
	return retval;
}

static int inv_slave_read(struct mldl_cfg *mldl_cfg,
			  void *gyro_adapter,
			  void *slave_adapter,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata,
			  void __user *usr_data)
{
	int retval;
	unsigned char *data;
	data = kzalloc(slave->read_len, GFP_KERNEL);
	if (!data)
		return -EFAULT;

	retval = inv_mpu_slave_read(mldl_cfg, gyro_adapter, slave_adapter,
				    slave, pdata, data);

	if ((!retval) &&
	    (copy_to_user((unsigned char __user *)usr_data,
			  data, slave->read_len)))
		retval = -EFAULT;

	kfree(data);
	return retval;
}

#ifndef CONFIG_MACH_STAR
static int mpu_handle_mlsl(void *sl_handle,
			   unsigned char addr,
			   unsigned int cmd,
			   struct mpu_read_write __user *usr_msg)
{
	int retval = 0;
	struct mpu_read_write msg;
	unsigned char *user_data;
	retval = copy_from_user(&msg, usr_msg, sizeof(msg));
	if (retval)
		return -EFAULT;

	user_data = msg.data;
	if (msg.length && msg.data) {
		unsigned char *data;
		data = kmalloc(msg.length, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)msg.data, msg.length);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		msg.data = data;
	} else {
		return -EPERM;
	}

	switch (cmd) {
	case MPU_READ:
		retval = inv_serial_read(sl_handle, addr,
					 msg.address, msg.length, msg.data);
		break;
	case MPU_WRITE:
		retval = inv_serial_write(sl_handle, addr,
					  msg.length, msg.data);
		break;
	case MPU_READ_MEM:
		retval = inv_serial_read_mem(sl_handle, addr,
					     msg.address, msg.length, msg.data);
		break;
	case MPU_WRITE_MEM:
		retval = inv_serial_write_mem(sl_handle, addr,
					      msg.address, msg.length,
					      msg.data);
		break;
	case MPU_READ_FIFO:
		retval = inv_serial_read_fifo(sl_handle, addr,
					      msg.length, msg.data);
		break;
	case MPU_WRITE_FIFO:
		retval = inv_serial_write_fifo(sl_handle, addr,
					       msg.length, msg.data);
		break;

	};
	if (retval) {
		dev_err(&((struct i2c_adapter *)sl_handle)->dev,
			"%s: i2c %d error %d\n",
			__func__, cmd, retval);
		kfree(msg.data);
		return retval;
	}
	retval = copy_to_user((unsigned char __user *)user_data,
			      msg.data, msg.length);
	kfree(msg.data);
	return retval;
}
#endif

/* ioctl - I/O control */
static long mpu_dev_ioctl(struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	struct i2c_client *client = mpu->client;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	int retval = 0;
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct ext_slave_descr **slave = mldl_cfg->slave;
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
	int ii;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;

	retval = mutex_lock_interruptible(&mpu->mutex);
	if (retval) {
		dev_err(&client->adapter->dev,
			"%s: mutex_lock_interruptible returned %d\n",
			__func__, retval);
		return retval;
	}

	switch (cmd) {
#if defined (CONFIG_MACH_STAR)
	case I2C_RDWR:
		mpu_dev_ioctl_rdrw(client, arg);
		break;
	case I2C_SLAVE:
		if ((arg & 0x7E) != (client->addr & 0x7E)) {
			dev_err(&client->adapter->dev,
				"%s: Invalid I2C_SLAVE arg %lu\n",
				__func__, arg);
		}
		break;
	case MPU_SET_MPU_CONFIG:
		retval = inv_mpu_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_MPU_CONFIG:
		retval = inv_mpu_get_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_ACCEL:
		retval = slave_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave[EXT_SLAVE_TYPE_ACCEL],
			pdata_slave[EXT_SLAVE_TYPE_ACCEL],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_COMPASS:
		retval = slave_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave[EXT_SLAVE_TYPE_COMPASS],
			pdata_slave[EXT_SLAVE_TYPE_COMPASS],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_PRESSURE:
		retval = slave_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			slave[EXT_SLAVE_TYPE_PRESSURE],
			pdata_slave[EXT_SLAVE_TYPE_PRESSURE],
			(struct ext_slave_config __user *)arg);
		break;

	case MPU_GET_CONFIG_ACCEL:
		retval = slave_get_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave[EXT_SLAVE_TYPE_ACCEL],
			pdata_slave[EXT_SLAVE_TYPE_ACCEL],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_CONFIG_COMPASS:
		retval = slave_get_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave[EXT_SLAVE_TYPE_COMPASS],
			pdata_slave[EXT_SLAVE_TYPE_COMPASS],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_CONFIG_PRESSURE:
		retval = slave_get_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			slave[EXT_SLAVE_TYPE_PRESSURE],
			pdata_slave[EXT_SLAVE_TYPE_PRESSURE],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_SUSPEND:
		{
		unsigned long sensors;
		sensors = ~(mldl_cfg->inv_mpu_cfg->requested_sensors);
		retval = inv_mpu_suspend(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			sensors);
		break;
		}
	case MPU_RESUME:
		{
		unsigned long sensors;
		sensors = mldl_cfg->inv_mpu_cfg->requested_sensors;
		retval = inv_mpu_resume(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			sensors);
		break;
		}
	case MPU_READ_ACCEL:
		retval = inv_slave_read(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave[EXT_SLAVE_TYPE_ACCEL],
			pdata_slave[EXT_SLAVE_TYPE_ACCEL],
			(unsigned char __user *)arg);
		break;
	case MPU_READ_COMPASS:
		retval = inv_slave_read(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave[EXT_SLAVE_TYPE_COMPASS],
			pdata_slave[EXT_SLAVE_TYPE_COMPASS],
			(unsigned char __user *)arg);
		break;
	case MPU_READ_PRESSURE:
		retval = inv_slave_read(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			slave[EXT_SLAVE_TYPE_PRESSURE],
			pdata_slave[EXT_SLAVE_TYPE_PRESSURE],
			(unsigned char __user *)arg);
		break;
#else
	case MPU_GET_EXT_SLAVE_PLATFORM_DATA:
		retval = mpu_dev_ioctl_get_ext_slave_platform_data(
			client,
			(struct ext_slave_platform_data __user *)arg);
		break;
	case MPU_GET_MPU_PLATFORM_DATA:
		retval = mpu_dev_ioctl_get_mpu_platform_data(
			client,
			(struct mpu_platform_data __user *)arg);
		break;
	case MPU_GET_EXT_SLAVE_DESCR:
		retval = mpu_dev_ioctl_get_ext_slave_descr(
			client,
			(struct ext_slave_descr __user *)arg);
		break;
	case MPU_READ:
	case MPU_WRITE:
	case MPU_READ_MEM:
	case MPU_WRITE_MEM:
	case MPU_READ_FIFO:
	case MPU_WRITE_FIFO:
		retval = mpu_handle_mlsl(
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			mldl_cfg->mpu_chip_info->addr, cmd,
			(struct mpu_read_write __user *)arg);
		break;
	case MPU_CONFIG_GYRO:
		retval = inv_mpu_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_ACCEL:
		retval = slave_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave[EXT_SLAVE_TYPE_ACCEL],
			pdata_slave[EXT_SLAVE_TYPE_ACCEL],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_COMPASS:
		retval = slave_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave[EXT_SLAVE_TYPE_COMPASS],
			pdata_slave[EXT_SLAVE_TYPE_COMPASS],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_PRESSURE:
		retval = slave_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			slave[EXT_SLAVE_TYPE_PRESSURE],
			pdata_slave[EXT_SLAVE_TYPE_PRESSURE],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_CONFIG_GYRO:
		retval = inv_mpu_get_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_CONFIG_ACCEL:
		retval = slave_get_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave[EXT_SLAVE_TYPE_ACCEL],
			pdata_slave[EXT_SLAVE_TYPE_ACCEL],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_CONFIG_COMPASS:
		retval = slave_get_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave[EXT_SLAVE_TYPE_COMPASS],
			pdata_slave[EXT_SLAVE_TYPE_COMPASS],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_CONFIG_PRESSURE:
		retval = slave_get_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			slave[EXT_SLAVE_TYPE_PRESSURE],
			pdata_slave[EXT_SLAVE_TYPE_PRESSURE],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_SUSPEND:
		retval = inv_mpu_suspend(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			arg);
		break;
	case MPU_RESUME:
		retval = inv_mpu_resume(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			arg);
		break;
	case MPU_PM_EVENT_HANDLED:
		dev_dbg(&client->adapter->dev, "%s: %d\n", __func__, cmd);
		complete(&mpu->completion);
		break;
	case MPU_READ_ACCEL:
		retval = inv_slave_read(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave[EXT_SLAVE_TYPE_ACCEL],
			pdata_slave[EXT_SLAVE_TYPE_ACCEL],
			(unsigned char __user *)arg);
		break;
	case MPU_READ_COMPASS:
		retval = inv_slave_read(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave[EXT_SLAVE_TYPE_COMPASS],
			pdata_slave[EXT_SLAVE_TYPE_COMPASS],
			(unsigned char __user *)arg);
		break;
	case MPU_READ_PRESSURE:
		retval = inv_slave_read(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			slave[EXT_SLAVE_TYPE_PRESSURE],
			pdata_slave[EXT_SLAVE_TYPE_PRESSURE],
			(unsigned char __user *)arg);
		break;
	case MPU_GET_REQUESTED_SENSORS:
		if (copy_to_user(
			   (__u32 __user *)arg,
			   &mldl_cfg->inv_mpu_cfg->requested_sensors,
			   sizeof(mldl_cfg->inv_mpu_cfg->requested_sensors)))
			retval = -EFAULT;
		break;
	case MPU_SET_REQUESTED_SENSORS:
		mldl_cfg->inv_mpu_cfg->requested_sensors = arg;
		break;
	case MPU_GET_IGNORE_SYSTEM_SUSPEND:
		if (copy_to_user(
			(unsigned char __user *)arg,
			&mldl_cfg->inv_mpu_cfg->ignore_system_suspend,
			sizeof(mldl_cfg->inv_mpu_cfg->ignore_system_suspend)))
			retval = -EFAULT;
		break;
	case MPU_SET_IGNORE_SYSTEM_SUSPEND:
		mldl_cfg->inv_mpu_cfg->ignore_system_suspend = arg;
		break;
	case MPU_GET_MLDL_STATUS:
		if (copy_to_user(
			(unsigned char __user *)arg,
			&mldl_cfg->inv_mpu_state->status,
			sizeof(mldl_cfg->inv_mpu_state->status)))
			retval = -EFAULT;
		break;
	case MPU_GET_I2C_SLAVES_ENABLED:
		if (copy_to_user(
			(unsigned char __user *)arg,
			&mldl_cfg->inv_mpu_state->i2c_slaves_enabled,
			sizeof(mldl_cfg->inv_mpu_state->i2c_slaves_enabled)))
			retval = -EFAULT;
		break;
#endif
	default:
		dev_err(&client->adapter->dev,
			"%s: Unknown cmd %x, arg %lu\n",
			__func__, cmd, arg);
		retval = -EINVAL;
	};

	mutex_unlock(&mpu->mutex);
	dev_dbg(&client->adapter->dev, "%s: %08x, %08lx, %d\n",
		__func__, cmd, arg, retval);

	if (retval > 0)
		retval = -retval;

	return retval;
}

#if defined (CONFIG_MACH_STAR)
#ifdef CONFIG_HAS_EARLYSUSPEND
void mpu3050_early_suspend(struct early_suspend *h)
{
	struct mpu_private_data *mpu = container_of(h,
						    struct
						    mpu_private_data,
						    early_suspend);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
	int ii;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = mpu->client->adapter;

	mutex_lock(&mpu->mutex);
	if (!mldl_cfg->inv_mpu_cfg->ignore_system_suspend) {
		dev_dbg(&mpu->client->adapter->dev,
			"%s: suspending\n", __func__);
		(void)inv_mpu_suspend(mldl_cfg,
				slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
				slave_adapter[EXT_SLAVE_TYPE_ACCEL],
				slave_adapter[EXT_SLAVE_TYPE_COMPASS],
				slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
				INV_ALL_SENSORS);
	} else {
		dev_dbg(&mpu->client->adapter->dev,
			"%s: Already suspended\n", __func__);
	}
	mutex_unlock(&mpu->mutex);
}

void mpu3050_early_resume(struct early_suspend *h)
{
	struct mpu_private_data *mpu = container_of(h,
						    struct
						    mpu_private_data,
						    early_suspend);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
	int ii;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = mpu->client->adapter;

	mutex_lock(&mpu->mutex);
	if (mpu->pid && !mldl_cfg->inv_mpu_cfg->ignore_system_suspend) {
		(void)inv_mpu_resume(mldl_cfg,
				slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
				slave_adapter[EXT_SLAVE_TYPE_ACCEL],
				slave_adapter[EXT_SLAVE_TYPE_COMPASS],
				slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
				mldl_cfg->inv_mpu_cfg->requested_sensors);
		dev_dbg(&mpu->client->adapter->dev,
			"%s for pid %d\n", __func__, mpu->pid);
	}
	mutex_unlock(&mpu->mutex);
}
#endif

static void mpu3050_power_terminate(void);
#endif
void mpu_shutdown(struct i2c_client *client)
{
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
	int ii;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;

	mutex_lock(&mpu->mutex);
	(void)inv_mpu_suspend(mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			INV_ALL_SENSORS);
#if defined (CONFIG_MACH_STAR)
	mpu3050_power_terminate();
#endif
	mutex_unlock(&mpu->mutex);
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
}

int mpu_dev_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
	int ii;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;

	mutex_lock(&mpu->mutex);
	if (!mldl_cfg->inv_mpu_cfg->ignore_system_suspend) {
		dev_dbg(&client->adapter->dev,
			"%s: suspending on event %d\n", __func__, mesg.event);
		(void)inv_mpu_suspend(mldl_cfg,
				slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
				slave_adapter[EXT_SLAVE_TYPE_ACCEL],
				slave_adapter[EXT_SLAVE_TYPE_COMPASS],
				slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
				INV_ALL_SENSORS);
	} else {
		dev_dbg(&client->adapter->dev,
			"%s: Already suspended %d\n", __func__, mesg.event);
	}
	mutex_unlock(&mpu->mutex);
	return 0;
}

int mpu_dev_resume(struct i2c_client *client)
{
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
	int ii;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;

	mutex_lock(&mpu->mutex);
	if (mpu->pid && !mldl_cfg->inv_mpu_cfg->ignore_system_suspend) {
		(void)inv_mpu_resume(mldl_cfg,
				slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
				slave_adapter[EXT_SLAVE_TYPE_ACCEL],
				slave_adapter[EXT_SLAVE_TYPE_COMPASS],
				slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
				mldl_cfg->inv_mpu_cfg->requested_sensors);
		dev_dbg(&client->adapter->dev,
			"%s for pid %d\n", __func__, mpu->pid);
	}
	mutex_unlock(&mpu->mutex);
	return 0;
}

/* define which file operations are supported */
static const struct file_operations mpu_fops = {
	.owner = THIS_MODULE,
	.read = mpu_read,
	.poll = mpu_poll,
	.unlocked_ioctl = mpu_dev_ioctl,
	.open = mpu_dev_open,
	.release = mpu_release,
};

int inv_mpu_register_slave(struct module *slave_module,
			struct i2c_client *slave_client,
			struct ext_slave_platform_data *slave_pdata,
			struct ext_slave_descr *(*get_slave_descr)(void))
{
	struct mpu_private_data *mpu = mpu_private_data;
	struct mldl_cfg *mldl_cfg;
	struct ext_slave_descr *slave_descr;
	struct ext_slave_platform_data **pdata_slave;
	char *irq_name = NULL;
	int result = 0;

	if (!slave_client || !slave_pdata || !get_slave_descr)
		return -EINVAL;

	if (!mpu) {
		dev_err(&slave_client->adapter->dev,
			"%s: Null mpu_private_data\n", __func__);
		return -EINVAL;
	}
	mldl_cfg    = &mpu->mldl_cfg;
	pdata_slave = mldl_cfg->pdata_slave;
	slave_descr = get_slave_descr();

	if (!slave_descr) {
		dev_err(&slave_client->adapter->dev,
			"%s: Null ext_slave_descr\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&mpu->mutex);
	if (mpu->pid) {
		mutex_unlock(&mpu->mutex);
		return -EBUSY;
	}

	if (pdata_slave[slave_descr->type]) {
		result = -EBUSY;
		goto out_unlock_mutex;
	}

	slave_pdata->address	= slave_client->addr;
	slave_pdata->irq	= slave_client->irq;
	slave_pdata->adapt_num	= i2c_adapter_id(slave_client->adapter);

	dev_info(&slave_client->adapter->dev,
		"%s: +%s Type %d: Addr: %2x IRQ: %2d, Adapt: %2d\n",
		__func__,
		slave_descr->name,
		slave_descr->type,
		slave_pdata->address,
		slave_pdata->irq,
		slave_pdata->adapt_num);

	switch (slave_descr->type) {
	case EXT_SLAVE_TYPE_ACCEL:
		irq_name = "accelirq";
		break;
	case EXT_SLAVE_TYPE_COMPASS:
		irq_name = "compassirq";
		break;
	case EXT_SLAVE_TYPE_PRESSURE:
		irq_name = "pressureirq";
		break;
	default:
		irq_name = "none";
	};
	if (slave_descr->init) {
		result = slave_descr->init(slave_client->adapter,
					slave_descr,
					slave_pdata);
		if (result) {
			dev_err(&slave_client->adapter->dev,
				"%s init failed %d\n",
				slave_descr->name, result);
			goto out_unlock_mutex;
		}
	}

	pdata_slave[slave_descr->type] = slave_pdata;
	mpu->slave_modules[slave_descr->type] = slave_module;
	mldl_cfg->slave[slave_descr->type] = slave_descr;

	goto out_unlock_mutex;

out_unlock_mutex:
	mutex_unlock(&mpu->mutex);

	if (!result && irq_name && (slave_pdata->irq > 0)) {
		int warn_result;
		dev_info(&slave_client->adapter->dev,
			"Installing %s irq using %d\n",
			irq_name,
			slave_pdata->irq);
		warn_result = slaveirq_init(slave_client->adapter,
					slave_pdata, irq_name);
		if (result)
			dev_WARN(&slave_client->adapter->dev,
				"%s irq assigned error: %d\n",
				slave_descr->name, warn_result);
	} else {
		dev_info(&slave_client->adapter->dev,
			"%s irq not assigned: %d %d %d\n",
			slave_descr->name,
			result, (int)irq_name, slave_pdata->irq);
	}

	return result;
}
EXPORT_SYMBOL(inv_mpu_register_slave);

void inv_mpu_unregister_slave(struct i2c_client *slave_client,
			struct ext_slave_platform_data *slave_pdata,
			struct ext_slave_descr *(*get_slave_descr)(void))
{
	struct mpu_private_data *mpu = mpu_private_data;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct ext_slave_descr *slave_descr;
	int result;

	dev_info(&slave_client->adapter->dev, "%s\n", __func__);

	if (!slave_client || !slave_pdata || !get_slave_descr)
		return;

	if (slave_pdata->irq)
		slaveirq_exit(slave_pdata);

	slave_descr = get_slave_descr();
	if (!slave_descr)
		return;

	mutex_lock(&mpu->mutex);

	if (slave_descr->exit) {
		result = slave_descr->exit(slave_client->adapter,
					slave_descr,
					slave_pdata);
		if (result)
			dev_err(&slave_client->adapter->dev,
				"Accel exit failed %d\n", result);
	}
	mldl_cfg->slave[slave_descr->type] = NULL;
	mldl_cfg->pdata_slave[slave_descr->type] = NULL;
	mpu->slave_modules[slave_descr->type] = NULL;

	mutex_unlock(&mpu->mutex);

}
EXPORT_SYMBOL(inv_mpu_unregister_slave);

static unsigned short normal_i2c[] = { I2C_CLIENT_END };

static const struct i2c_device_id mpu_id[] = {
	{"mpu3050", 0},
	{"mpu6050", 0},
	{"mpu6050_no_accel", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, mpu_id);

int mpu_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
	struct mpu_platform_data *pdata;
	struct mpu_private_data *mpu;
	struct mldl_cfg *mldl_cfg;
	int res = 0;
	int ii = 0;
	unsigned long irq_flags;

	dev_info(&client->adapter->dev, "%s: %d\n", __func__, ii++);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		res = -ENODEV;
		goto out_check_functionality_failed;
	}

	mpu = kzalloc(sizeof(struct mpu_private_data), GFP_KERNEL);
	if (!mpu) {
		res = -ENOMEM;
		goto out_alloc_data_failed;
	}
	mldl_cfg = &mpu->mldl_cfg;
	mldl_cfg->mpu_ram	= &mpu->mpu_ram;
	mldl_cfg->mpu_gyro_cfg	= &mpu->mpu_gyro_cfg;
	mldl_cfg->mpu_offsets	= &mpu->mpu_offsets;
	mldl_cfg->mpu_chip_info	= &mpu->mpu_chip_info;
	mldl_cfg->inv_mpu_cfg	= &mpu->inv_mpu_cfg;
	mldl_cfg->inv_mpu_state	= &mpu->inv_mpu_state;

	mldl_cfg->mpu_ram->length = MPU_MEM_NUM_RAM_BANKS * MPU_MEM_BANK_SIZE;
	mldl_cfg->mpu_ram->ram = kzalloc(mldl_cfg->mpu_ram->length, GFP_KERNEL);
	if (!mldl_cfg->mpu_ram->ram) {
		res = -ENOMEM;
		goto out_alloc_ram_failed;
	}
	mpu_private_data = mpu;
	i2c_set_clientdata(client, mpu);
	mpu->client = client;

	init_waitqueue_head(&mpu->mpu_event_wait);
	mutex_init(&mpu->mutex);
	init_completion(&mpu->completion);

	mpu->response_timeout = 60;	/* Seconds */
	mpu->timeout.function = mpu_pm_timeout;
	mpu->timeout.data = (u_long) mpu;
	init_timer(&mpu->timeout);

#ifndef CONFIG_MACH_STAR
	mpu->nb.notifier_call = mpu_pm_notifier_callback;
	mpu->nb.priority = 0;
	res = register_pm_notifier(&mpu->nb);
	if (res) {
		dev_err(&client->adapter->dev,
			"Unable to register pm_notifier %d\n", res);
		goto out_register_pm_notifier_failed;
	}
#endif

	pdata = (struct mpu_platform_data *)client->dev.platform_data;
	if (!pdata) {
		dev_WARN(&client->adapter->dev,
			 "Missing platform data for mpu\n");
	}
	mldl_cfg->pdata = pdata;

	mldl_cfg->mpu_chip_info->addr = client->addr;
	res = inv_mpu_open(&mpu->mldl_cfg, client->adapter, NULL, NULL, NULL);

	if (res) {
		dev_err(&client->adapter->dev,
			"Unable to open %s %d\n", MPU_NAME, res);
		res = -ENODEV;
		goto out_whoami_failed;
	}

	mpu->dev.minor = MISC_DYNAMIC_MINOR;
	mpu->dev.name = "mpu";
	mpu->dev.fops = &mpu_fops;
	res = misc_register(&mpu->dev);
	if (res < 0) {
		dev_err(&client->adapter->dev,
			"ERROR: misc_register returned %d\n", res);
		goto out_misc_register_failed;
	}

	if (client->irq) {
		dev_info(&client->adapter->dev,
			 "Installing irq using %d\n", client->irq);
		if (BIT_ACTL_LOW == ((mldl_cfg->pdata->int_config) & BIT_ACTL))
			irq_flags = IRQF_TRIGGER_FALLING;
		else
			irq_flags = IRQF_TRIGGER_RISING;
		res = mpuirq_init(client, mldl_cfg, irq_flags);

		if (res)
			goto out_mpuirq_failed;
	} else {
		dev_WARN(&client->adapter->dev,
			 "Missing %s IRQ\n", MPU_NAME);
	}
	return res;

out_mpuirq_failed:
	misc_deregister(&mpu->dev);
out_misc_register_failed:
	inv_mpu_close(&mpu->mldl_cfg, client->adapter, NULL, NULL, NULL);
out_whoami_failed:
	unregister_pm_notifier(&mpu->nb);
out_register_pm_notifier_failed:
	kfree(mldl_cfg->mpu_ram->ram);
	mpu_private_data = NULL;
out_alloc_ram_failed:
	kfree(mpu);
out_alloc_data_failed:
out_check_functionality_failed:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, res);
	return res;

}

static int mpu_remove(struct i2c_client *client)
{
	struct mpu_private_data *mpu = i2c_get_clientdata(client);
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
	int ii;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}

	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;
	dev_dbg(&client->adapter->dev, "%s\n", __func__);

	inv_mpu_close(mldl_cfg,
		slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
		slave_adapter[EXT_SLAVE_TYPE_ACCEL],
		slave_adapter[EXT_SLAVE_TYPE_COMPASS],
		slave_adapter[EXT_SLAVE_TYPE_PRESSURE]);


	if (client->irq)
		mpuirq_exit();

	misc_deregister(&mpu->dev);

	unregister_pm_notifier(&mpu->nb);

	kfree(mpu->mldl_cfg.mpu_ram->ram);
	kfree(mpu);

	return 0;
}

static struct i2c_driver mpu_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = mpu_probe,
	.remove = mpu_remove,
	.id_table = mpu_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = MPU_NAME,
		   },
	.address_list = normal_i2c,
	.shutdown = mpu_shutdown,	/* optional */
	.suspend = mpu_dev_suspend,	/* optional */
	.resume = mpu_dev_resume,	/* optional */

};

#if defined (CONFIG_MACH_STAR)
#include <linux/delay.h>
#include <../../../arch/arm/mach-tegra/gpio-names.h>
#include <linux/regulator/consumer.h>

static void mpu3050_power_init(void)
{
	struct regulator *regulator1 = NULL;
	struct regulator *regulator2 = NULL;
	unsigned int gyro_int_gpio;

	gyro_int_gpio = TEGRA_GPIO_PQ5;
	gpio_request(gyro_int_gpio, "gyro_int");
	gpio_direction_output(gyro_int_gpio, 0);
	tegra_gpio_enable(gyro_int_gpio);

	regulator1 = regulator_get(NULL, "vcc_sensor_3v0");
	if (!regulator1) {
		printk(KERN_INFO "mpu3050: vcc_sensor_3v0 failed\n");
	}

	regulator_set_voltage(regulator1, 3000000, 3000000);

	regulator2 = regulator_get(NULL, "vcc_sensor_1v8");
	if (!regulator2) {
		printk(KERN_INFO "mpu3050: vcc_sensor_1v8 failed\n");
	}
	regulator_set_voltage(regulator2, 1800000, 1800000);

	regulator_enable(regulator1);
	mdelay(10);
	regulator_enable(regulator2);
	mdelay(10);
	gpio_direction_input(gyro_int_gpio);	

	gpio_request(TEGRA_GPIO_PR4, "com_int");
	gpio_request(TEGRA_GPIO_PI0, "motion_int");
	gpio_direction_input(TEGRA_GPIO_PR4);
	gpio_direction_input(TEGRA_GPIO_PI0);

	mdelay(1);
}

static void mpu3050_power_terminate(void)
{
	struct regulator *regulator1 = NULL;
	struct regulator *regulator2 = NULL;

	mdelay(10);
	regulator1 = regulator_get(NULL, "vcc_sensor_3v0");
	if (!regulator1) {
		printk(KERN_INFO "mpu3050: vcc_sensor_3v0 failed\n");
	}
	regulator_set_voltage(regulator1, 3000*1000, 3000*1000);
	regulator2 = regulator_get(NULL, "vcc_sensor_1v8");
	if (!regulator2) {
		printk(KERN_INFO "mpu3050: vcc_sensor_1v8 failed\n");
	}
	regulator_set_voltage(regulator2, 1800*1000, 1800*1000);

	regulator_disable(regulator2);	// 1.8 VI/O OFF
	mdelay(10);
	regulator_disable(regulator1);	// 3.0 VDD OFF

	gpio_free(TEGRA_GPIO_PR4);
	gpio_free(TEGRA_GPIO_PI0);
}
#endif

static int __init mpu_init(void)
{
#if defined (CONFIG_MACH_STAR)
	mpu3050_power_init();
#endif

	int res = i2c_add_driver(&mpu_driver);
	pr_info("%s: Probe name %s\n", __func__, MPU_NAME);
	if (res)
		pr_err("%s failed\n", __func__);
	return res;
}

static void __exit mpu_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&mpu_driver);
}

module_init(mpu_init);
module_exit(mpu_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("User space character device interface for MPU");
MODULE_LICENSE("GPL");
MODULE_ALIAS(MPU_NAME);
