#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpu.h>

bool single_core_mode = false;
EXPORT_SYMBOL(single_core_mode);

bool auto_hotplug_enabled = false;
EXPORT_SYMBOL(auto_hotplug_enabled);

static ssize_t single_core_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (single_core_mode ? 1 : 0));
}

static ssize_t single_core_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int cpu;
	unsigned long data;

	data = simple_strtoul(buf, NULL, 10);

	if(data) {
		if (! single_core_mode) {
			for_each_possible_cpu(cpu) {
				if (likely(cpu_online(cpu) && (cpu))) {
					cpu_down(cpu);
				}
			}
			single_core_mode = true;
		}
	} else {
		if (single_core_mode) {
			for_each_possible_cpu(cpu) {
				if (likely(!cpu_online(cpu))) {
					cpu_up(cpu);
				}
			}
			single_core_mode = false;
		}
	}
	return count;
}

static ssize_t auto_hotplug_enabled_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (auto_hotplug_enabled ? 1 : 0));
}

static ssize_t auto_hotplug_enabled_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;

	data = simple_strtoul(buf, NULL, 10);

	if(data) {
		if (! auto_hotplug_enabled)
			auto_hotplug_enabled = true;
	} else {
		if (auto_hotplug_enabled)
			auto_hotplug_enabled = false;
	}

	return count;
}

static struct kobj_attribute auto_hotplug_enabled_attribute =
	__ATTR(auto_hotplug_enabled, 0666, auto_hotplug_enabled_show, auto_hotplug_enabled_store);

static struct kobj_attribute single_core_mode_attribute =
	__ATTR(single_core_mode, 0666, single_core_mode_show, single_core_mode_store);

static struct attribute *cpu_control_attrs[] = {
	&single_core_mode_attribute.attr,
	&auto_hotplug_enabled_attribute.attr,
	NULL,
};

static struct attribute_group cpu_control_attrs_group = {
	.attrs = cpu_control_attrs,
};

static struct kobject *cpu_control_kobj;

int __init cpu_control_init(void)
{
	int res;

	cpu_control_kobj = kobject_create_and_add("auto_hotplug", kernel_kobj);
	if (!cpu_control_kobj) {
		pr_err("%s auto_hotplug kobject create failed!\n", __FUNCTION__);
		return -ENOMEM;
	}

	res = sysfs_create_group(cpu_control_kobj, &cpu_control_attrs_group);

	if (res) {
		pr_info("%s auto_hotplug sysfs create failed!\n", __FUNCTION__);
		kobject_put(cpu_control_kobj);
	}

	return res;
}
late_initcall(cpu_control_init);
