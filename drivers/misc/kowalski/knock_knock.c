#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpu.h>

bool kowalski_knock_knock_wake = true;
EXPORT_SYMBOL(kowalski_knock_knock_wake);

bool kowalski_knock_knock_sleep = true;
EXPORT_SYMBOL(kowalski_knock_knock_sleep);

static ssize_t knock_knock_sleep_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (kowalski_knock_knock_sleep ? 1 : 0));
}

static ssize_t knock_knock_sleep_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;

	data = simple_strtoul(buf, NULL, 10);

	if(data)
		kowalski_knock_knock_sleep = true;
	else
		kowalski_knock_knock_sleep = false;

	return count;
}

static ssize_t knock_knock_wake_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (kowalski_knock_knock_wake ? 1 : 0));
}

static ssize_t knock_knock_wake_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;

	data = simple_strtoul(buf, NULL, 10);

	if(data)
		kowalski_knock_knock_wake = true;
	else
		kowalski_knock_knock_wake = false;

	return count;
}

static struct kobj_attribute knock_knock_wake_attribute =
	__ATTR(knock_knock_wake, 0666, knock_knock_wake_show, knock_knock_wake_store);

static struct kobj_attribute knock_knock_sleep_attribute =
	__ATTR(knock_knock_sleep, 0666, knock_knock_sleep_show, knock_knock_sleep_store);

static struct attribute *control_attrs[] = {
	&knock_knock_sleep_attribute.attr,
	&knock_knock_wake_attribute.attr,
	NULL,
};

static struct attribute_group control_attrs_group = {
	.attrs = control_attrs,
};

static struct kobject *control_kobj;

int __init control_init(void)
{
	int res;

	control_kobj = kobject_create_and_add("knock_knock", kernel_kobj);
	if (!control_kobj) {
		pr_err("%s knock_knock kobject create failed!\n", __FUNCTION__);
		return -ENOMEM;
	}

	res = sysfs_create_group(control_kobj, &control_attrs_group);

	if (res) {
		pr_info("%s knock_knock sysfs create failed!\n", __FUNCTION__);
		kobject_put(control_kobj);
	}

	return res;
}
late_initcall(control_init);
