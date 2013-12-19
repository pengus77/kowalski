/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>

static const char* regulator_path = "/sys/class/regulator/regulator.27";
static const unsigned int uv_step = 10000;

static ssize_t uv_step_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", uv_step);
}

static ssize_t regulator_path_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", regulator_path);
}

static struct kobj_attribute uv_step_attribute =
	__ATTR(uv_step, 0444, uv_step_show, NULL);

static struct kobj_attribute regulator_path_attribute =
	__ATTR(regulator_path, 0444, regulator_path_show, NULL);

static struct attribute *attrs[] = {
	&uv_step_attribute.attr,
	&regulator_path_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *uv_config_kobj;

static int __init uv_config_init(void)
{
	int retval;

	uv_config_kobj = kobject_create_and_add("uv_config", kernel_kobj);
	if (!uv_config_kobj) {
		return -ENOMEM;
	}

	retval = sysfs_create_group(uv_config_kobj, &attr_group);
	if (retval)
		kobject_put(uv_config_kobj);
		
	return retval;
}
/* end sysfs interface */

static void __exit uv_config_exit(void)
{
	kobject_put(uv_config_kobj);
}

module_init(uv_config_init);
module_exit(uv_config_exit);
