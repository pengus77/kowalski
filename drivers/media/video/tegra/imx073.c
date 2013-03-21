/*
 * imx073.c - imx073 sensor driver
 *
 * Copyright (C) 2011 Google Inc.
 *
 * Contributors:
 *      Rebecca Schultz Zavin <rebecca@android.com>
 *
 * Leverage OV9640.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/imx073.h>


struct imx073_reg {
	u16 addr;
	u16 val;
};

struct imx073_info {
	int mode;
	struct i2c_client *i2c_client;
	struct imx073_platform_data *pdata;
};

#define IMX073_TABLE_WAIT_MS 0
#define IMX073_TABLE_END 1
#define IMX073_MAX_RETRIES 3

static struct imx073_reg mode_3264x2448[] = {

	{0x0100, 0x00},
	{0x0307,	0x24},
	{0x302B,	0x38},
	{0x30E5,	0x04},
	{0x3300,	0x00},
	{0x0101,	0x03}, // imager_orientation
	{0x300A,	0x80},
	{0x3014,	0x08},
	{0x3015,	0x37},
	{0x3017,	0x60},
	{0x301C,	0x01},
	{0x3031,	0x28},
	{0x3040,	0x00},
	{0x3041,	0x60},
	{0x3047,	0x10},
	{0x3051,	0x24},
	{0x3053,	0x34},
	{0x3055,	0x3B},
	{0x3057,	0xC0},
	{0x3060,	0x30},
	{0x3065,	0x00},
	{0x30A1,	0x03},
	{0x30A3,	0x01},
	{0x30AA,	0x88},
	{0x30AB,	0x1C},
	{0x30B0,	0x32},
	{0x30B2,	0x83},
	{0x30D3,	0x04},
	{0x310C,	0xE9},
	{0x310D,	0x00},
	{0x316B,	0x14},
	{0x316D,	0x3B},
	{0x31A4,	0xD8},
	{0x31A6,	0x17},
	{0x31AC,	0xCF},
	{0x31AE,	0xF1},
	{0x31B4,	0xD8},
	{0x31B6,	0x17},
	{0x3302,	0x0A},
	{0x3303,	0x09},
	{0x3304,	0x05},
	{0x3305,	0x04},
	{0x3306,	0x15},
	{0x3307,	0x03},
	{0x3308,	0x13},
	{0x3309,	0x05},
	{0x330A,	0x0B},
	{0x330B,	0x04},
	{0x330C,	0x0B},
	{0x330D,	0x06},
	{0x0340,	0x09},
	{0x0341,	0xCE},
	{0x0342,  0x0d},
	{0x0343,  0x70},
	{0x0344,	0x00},	// x_addr_start
	{0x0345,	0x00},
	{0x0346,	0x00},	// y_addr_start
	{0x0347,	0x00},
	{0x0348,	0x0c},	// x_addr_end
	{0x0349,	0xcf},
	{0x034A,	0x09},	// y_addr_end
	{0x034B,	0x9f},

	{0x034C,	0x0C},	// 3280 
	{0x034D,	0xD0},
	{0x034E,	0x09},	// 2464
	{0x034F,	0xA0},
	{0x0381,	0x01},
	{0x0383,	0x01},
	{0x0385,	0x01},
	{0x0387,	0x01},
	{0x3001,	0x00},
	{0x3016,	0x06},
	{0x30E8,	0x06},
	{0x3301,	0x00},
	{0x0100,	0x01},
	{IMX073_TABLE_END, 0x0000}
};

static struct imx073_reg mode_3264x1224[] = {
	{0x0100, 0x00},
	{0x0307,	0x24},
	{0x302B,	0x38},
	{0x30E5,	0x04},
	{0x3300,	0x00},
	{0x0101,	0x03},// imager_orientation
	{0x300A,	0x80},
	{0x3014,	0x08},
	{0x3015,	0x37},
	{0x3017,	0x60},
	{0x301C,	0x01},
	{0x3031,	0x28},
	{0x3040,	0x00},
	{0x3041,	0x60},
	{0x3047,	0x10},
	{0x3051,	0x24},
	{0x3053,	0x34},
	{0x3055,	0x3B},
	{0x3057,	0xC0},
	{0x3060,	0x30},
	{0x3065,	0x00},
	{0x30A1,	0x03},
	{0x30A3,	0x01},
	{0x30AA,	0x88},
	{0x30AB,	0x1C},
	{0x30B0,	0x32},
	{0x30B2,	0x83},
	{0x30D3,	0x04},
	{0x310C,	0xE9},
	{0x310D,	0x00},
	{0x316B,	0x14},
	{0x316D,	0x3B},
	{0x31A4,	0xD8},
	{0x31A6,	0x17},
	{0x31AC,	0xCF},
	{0x31AE,	0xF1},
	{0x31B4,	0xD8},
	{0x31B6,	0x17},
	{0x3302,	0x0A},
	{0x3303,	0x09},
	{0x3304,	0x05},
	{0x3305,	0x04},
	{0x3306,	0x15},
	{0x3307,	0x03},
	{0x3308,	0x13},
	{0x3309,	0x05},
	{0x330A,	0x0B},
	{0x330B,	0x04},
	{0x330C,	0x0B},
	{0x330D,	0x06},
	{0x0340,	0x04}, // pg 66, frame length lines 
	{0x0341,	0xE6}, //
	{0x0342,  0x0d},
	{0x0343,  0x70},

	{0x0344,	0x00},	// x_addr_start
	{0x0345,	0x00},
	{0x0346,	0x00},	// y_addr_start
	{0x0347,	0x00},
	{0x0348,	0x0c},	// x_addr_end
	{0x0349,	0xcf},
	{0x034A,	0x09},	// y_addr_end
	{0x034B,	0x9f},

	{0x034C,	0x0C}, // x output size
	{0x034D,	0xD0}, // x output size (3280)
	{0x034E,	0x04}, // pg 66, y output size (1232 lines, which is 22 more lines than 1210)
	{0x034F,	0xD0}, //
	{0x0381,	0x01}, // x_even_inc
	{0x0383,	0x01}, // x_odd_inc
	{0x0385,	0x01}, // y_even_inc
	{0x0387,	0x03}, // y_odd_inc
	{0x3001,	0x00},
	{0x3016,	0x46},
	{0x30E8,	0x06},
	{0x3301,	0x00},
	{0x0100,	0x01},

	{IMX073_TABLE_END, 0x0000}
};

static struct imx073_reg mode_2720x1530[] =
{ 
	{0x0100, 0x00},
	{0x0307,	0x24},
	{0x302B,	0x38},	
	{0x30E5,	0x04},
	{0x3300,	0x00},
	{0x0101,	0x03},

	{0x300A,	0x80},
	{0x3014,	0x08},
	{0x3015,	0x37},
	{0x3017,	0x60},
	{0x301C,	0x01},
	{0x3031,	0x28},
	{0x3040,	0x00},
	{0x3041,	0x60},
	{0x3047,	0x10},
	{0x3051,	0x24},
	{0x3053,	0x34},
	{0x3055,	0x3B},
	{0x3057,	0xC0},
	{0x3060,	0x30},
	{0x3065,	0x00},
	{0x30A1,	0x03},
	{0x30A3,	0x01},
	{0x30AA,	0x88},
	{0x30AB,	0x1C},
	{0x30B0,	0x32},
	{0x30B2,	0x83},
	{0x30D3,	0x04},
	{0x310C,	0xE9},
	{0x310D,	0x00},
	{0x316B,	0x14},
	{0x316D,	0x3B},
	{0x31A4,	0xD8},
	{0x31A6,	0x17},
	{0x31AC,	0xCF},
	{0x31AE,	0xF1},
	{0x31B4,	0xD8},
	{0x31B6,	0x17},
	{0x3302,	0x0A},
	{0x3303,	0x09},
	{0x3304,	0x05},
	{0x3305,	0x04},
	{0x3306,	0x15},
	{0x3307,	0x03},
	{0x3308,	0x13},
	{0x3309,	0x05},
	{0x330A,	0x0B},
	{0x330B,	0x04},
	{0x330C,	0x0B},
	{0x330D,	0x06},

	{0x0340,	0x06},	// frame_length
	{0x0341,	0x1f},

	{0x0344,	0x01},	// x_addr_start
	{0x0345,	0x18},
	{0x0346,	0x01},	// y_addr_start
	{0x0347,	0xd5},
	{0x0348,	0x0b},	// x_addr_end
	{0x0349,	0xb7},
	{0x034A,	0x07},	// y_addr_end
	{0x034B,	0xce},

	{0x034C,	0x0a},	// x_out size
	{0x034D,	0xa0},
	{0x034E,	0x05},	// y_out size
	{0x034F,	0xfa},
	{0x0381,	0x01},	// x_even_inc
	{0x0383,	0x01},	// x_odd_inc
	{0x0385,	0x01},	// y_even_inc
	{0x0387,	0x01},	// y_odd_inc
	{0x3001,	0x00},	// HMODE ADD
	{0x3016,	0x06},	// VMODE ADD
	{0x30E8,	0x06},	// HADDAVE
	{0x3301,	0x00},	// RGLANESEL

	{0x0100,	0x01},

	{IMX073_TABLE_END, 0x0000}
};


enum {
	IMX073_MODE_3264x2448,
	IMX073_MODE_3264x1224,
	IMX073_MODE_2720x1530,
};

static struct imx073_reg *mode_table[] = {
	[IMX073_MODE_3264x2448] = mode_3264x2448,
	[IMX073_MODE_3264x1224] = mode_3264x1224,
	[IMX073_MODE_2720x1530] = mode_2720x1530,
};

/* 2 regs to program frame length */
static inline void imx073_get_frame_length_regs(struct imx073_reg *regs,
		u32 frame_length)
{
	regs->addr = 0x0340;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x0341;
	(regs + 1)->val = (frame_length) & 0xff;
}

/* 3 regs to program coarse time */
static inline void imx073_get_coarse_time_regs(struct imx073_reg *regs,
		u32 coarse_time)
{
	regs->addr = 0x202;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = 0x203;
	(regs + 1)->val = (coarse_time) & 0xff;
}

/* 1 reg to program gain */
static inline void imx073_get_gain_reg(struct imx073_reg *regs, u16 gain)
{
	regs->addr = 0x205;
	regs->val = gain;
}

static int imx073_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = data[2];

	return 0;
}

static int imx073_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("imx073: i2c transfer failed, retrying %x %x\n",
				addr, val);
		msleep(3);
	} while (retry <= IMX073_MAX_RETRIES);

	return err;
}

static int imx073_write_table(struct i2c_client *client,
		const struct imx073_reg table[],
		const struct imx073_reg override_list[],
		int num_override_regs)
{
	int err;
	const struct imx073_reg *next;
	int i;
	u16 val;
	u16 addr;

	for (next = table; next->addr != IMX073_TABLE_END; next++) {
		if (next->addr == IMX073_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

		err = imx073_write_reg(client, next->addr, val);
		if (err)
			return err;
	}

	/* When an override list is passed in, replace the reg */
	/* value to write if the reg is in the list            */
	err = imx073_write_reg(client, 0x104, 0x01);
	if (err)
		return err;

	if (override_list) {
		for (i = 0; i < num_override_regs; i++) {
			addr = override_list[i].addr;
			val = override_list[i].val;
			//		}

			err = imx073_write_reg(client, addr, val);
			if (err)
				return err;
	}
}

err = imx073_write_reg(client, 0x104, 0x00);
if (err)
	return err;

	return 0;
	}

static int imx073_set_mode(struct imx073_info *info, struct imx073_mode *mode)
{
	int sensor_mode;
	int err;
	struct imx073_reg reg_list[6];

	memset(reg_list, 0, sizeof(struct imx073_reg));

	pr_info("%s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
			__func__, mode->xres, mode->yres, mode->frame_length,
			mode->coarse_time, mode->gain);
	if (mode->xres == 3264 && mode->yres == 2448)
		sensor_mode = IMX073_MODE_3264x2448;
	else if (mode->xres == 3264 && mode->yres == 1224)
		sensor_mode = IMX073_MODE_3264x1224;
	else if (mode->xres == 2720 && mode->yres == 1530)
		sensor_mode = IMX073_MODE_2720x1530;
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
				__func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	imx073_get_frame_length_regs(&reg_list[0], mode->frame_length);
	imx073_get_coarse_time_regs(&reg_list[2], mode->coarse_time);
	imx073_get_gain_reg(&reg_list[4], mode->gain);

	err = imx073_write_table(info->i2c_client, mode_table[sensor_mode],
			reg_list, 6);
	if (err)
		return err;

	info->mode = sensor_mode;
	return 0;
}

static int imx073_set_frame_length(struct imx073_info *info, u32 frame_length)
{
	struct imx073_reg reg_list[2];
	int i = 0;
	int ret;

	imx073_get_frame_length_regs(reg_list, frame_length);
	ret = imx073_write_reg(info->i2c_client, 0x0104, 0x01);
	if (ret)
		return ret;

	for (i = 0; i < 2; i++)	{
		ret = imx073_write_reg(info->i2c_client, reg_list[i].addr,
				reg_list[i].val);
		if (ret)
			return ret;
	}

	ret = imx073_write_reg(info->i2c_client, 0x0104, 0x0);
	if (ret)
		return ret;

	return 0;
}

static int imx073_set_coarse_time(struct imx073_info *info, u32 coarse_time)
{
	int ret;

	struct imx073_reg reg_list[3];
	int i = 0;

	imx073_get_coarse_time_regs(reg_list, coarse_time);

	ret = imx073_write_reg(info->i2c_client, 0x104, 0x01);
	if (ret)
		return ret;

	for (i = 0; i < 3; i++)	{
		ret = imx073_write_reg(info->i2c_client, reg_list[i].addr,
				reg_list[i].val);
		if (ret)
			return ret;
	}

	ret = imx073_write_reg(info->i2c_client, 0x104, 0x0);
	if (ret)
		return ret;

	return 0;
}

static int imx073_set_gain(struct imx073_info *info, u16 gain)
{
	int ret;
	struct imx073_reg reg_list;

	//pr_err("[Karl-imx073] Gain = %u\n", gain);

	imx073_get_gain_reg(&reg_list, gain);

	ret = imx073_write_reg(info->i2c_client, 0x104, 0x1);
	if (ret)
		return ret;

	ret = imx073_write_reg(info->i2c_client, reg_list.addr, reg_list.val);

	ret = imx073_write_reg(info->i2c_client, 0x104, 0x0);
	if (ret)
		return ret;

	return ret;
}

static int imx073_get_status(struct imx073_info *info, u8 *status)
{
	int err;

	*status = 0;
	err = imx073_read_reg(info->i2c_client, 0x205, status);
	pr_info("%s: status=%u err=%d\n", __func__, *status, err);
	return err;
}

static long imx073_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int err;
	struct imx073_info *info = file->private_data;

	switch (cmd) {
		case IMX073_IOCTL_SET_MODE:
			{
				struct imx073_mode mode;
				if (copy_from_user(&mode,
							(const void __user *)arg,
							sizeof(struct imx073_mode))) {
					pr_info("%s %d\n", __func__, __LINE__);
					return -EFAULT;
				}

				return imx073_set_mode(info, &mode);
			}
		case IMX073_IOCTL_SET_FRAME_LENGTH:
			return imx073_set_frame_length(info, (u32)arg);
		case IMX073_IOCTL_SET_COARSE_TIME:
			return imx073_set_coarse_time(info, (u32)arg);
		case IMX073_IOCTL_SET_GAIN:
			return imx073_set_gain(info, (u16)arg);
		case IMX073_IOCTL_GET_STATUS:
			{
				u8 status;

				err = imx073_get_status(info, &status);
				if (err)
					return err;
				if (copy_to_user((void __user *)arg, &status,
							2)) {
					pr_info("%s %d\n", __func__, __LINE__);
					return -EFAULT;
				}
				return 0;
			}
		default:
			return -EINVAL;
	}
	return 0;
}

static struct imx073_info *info;

static int imx073_open(struct inode *inode, struct file *file)
{
	u8 status;

	pr_info("%s\n", __func__);
	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	imx073_get_status(info, &status);
	return 0;
}

int imx073_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	file->private_data = NULL;
	return 0;
}


static const struct file_operations imx073_fileops = {
	.owner = THIS_MODULE,
	.open = imx073_open,
	.unlocked_ioctl = imx073_ioctl,
	.release = imx073_release,
};

static struct miscdevice imx073_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "imx073",
	.fops = &imx073_fileops,
};

static int imx073_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err;

	pr_info("imx073: probing sensor.\n");

	info = kzalloc(sizeof(struct imx073_info), GFP_KERNEL);
	if (!info) {
		pr_err("imx073: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&imx073_device);
	if (err) {
		pr_err("imx073: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);
	return 0;
}

static int imx073_remove(struct i2c_client *client)
{
	struct imx073_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&imx073_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id imx073_id[] = {
	{ "imx073", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, imx073_id);

static struct i2c_driver imx073_i2c_driver = {
	.driver = {
		.name = "imx073",
		.owner = THIS_MODULE,
	},
	.probe = imx073_probe,
	.remove = imx073_remove,
	.id_table = imx073_id,
};

static int __init imx073_init(void)
{
	pr_info("imx073 sensor driver loading\n");
	return i2c_add_driver(&imx073_i2c_driver);
}

static void __exit imx073_exit(void)
{
	i2c_del_driver(&imx073_i2c_driver);
}

module_init(imx073_init);
module_exit(imx073_exit);

