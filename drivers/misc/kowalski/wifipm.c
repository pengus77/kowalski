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
 * Created by Ivan Cerrato (pengus77) on 1/27/13.
 */

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/module.h>

#define DEFAULT_DBM 19
#define MAX_DBM 24

bool kowalski_wifi_max_pm = false;
bool kowalski_wifi_wake_pm = true;
bool kowalski_wifi_hotspot_pm = false;
int kowalski_wifi_dbm = DEFAULT_DBM;

/* module availability exports */
EXPORT_SYMBOL(kowalski_wifi_max_pm);
EXPORT_SYMBOL(kowalski_wifi_wake_pm);
EXPORT_SYMBOL(kowalski_wifi_hotspot_pm);
EXPORT_SYMBOL(kowalski_wifi_dbm);

typedef void (*dbm_callback_type)(int);
dbm_callback_type dbm_callback;

void kowalski_wifi_register_dbm_cb(void *cb) {
	dbm_callback = (dbm_callback_type)(cb);
}

void kowalski_wifi_unregister_dbm_cb(void) {
	dbm_callback = 0;
}

EXPORT_SYMBOL(kowalski_wifi_register_dbm_cb);
EXPORT_SYMBOL(kowalski_wifi_unregister_dbm_cb);

/* sysfs interface */
static ssize_t max_pm_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", kowalski_wifi_max_pm);
}

static ssize_t max_pm_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned long check = simple_strtoul(buf, NULL, 10);
	if (check)
		kowalski_wifi_max_pm = true;
	else
		kowalski_wifi_max_pm = false;
	return count;
}

static ssize_t wake_pm_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", kowalski_wifi_wake_pm);
}

static ssize_t wake_pm_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned long check = simple_strtoul(buf, NULL, 10);
	if (check)
		kowalski_wifi_wake_pm = true;
	else
		kowalski_wifi_wake_pm = false;
	return count;
}

static ssize_t hotspot_pm_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", kowalski_wifi_hotspot_pm);
}

static ssize_t hotspot_pm_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned long check = simple_strtoul(buf, NULL, 10);
	if (check)
		kowalski_wifi_hotspot_pm = true;
	else
		kowalski_wifi_hotspot_pm = false;
	return count;
}

static ssize_t dbm_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", kowalski_wifi_dbm);
}

static ssize_t dbm_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned long check = simple_strtoul(buf, NULL, 10);
	if (check && check <= MAX_DBM)
		kowalski_wifi_dbm = check;
	else
		kowalski_wifi_dbm = 19;

	if (dbm_callback)
		dbm_callback(kowalski_wifi_dbm);

	return count;
}

static struct kobj_attribute max_pm_attribute =
	__ATTR(max_pm, 0666, max_pm_show, max_pm_store);

static struct kobj_attribute wake_pm_attribute =
	__ATTR(wake_pm, 0666, wake_pm_show, wake_pm_store);

static struct kobj_attribute hotspot_pm_attribute =
	__ATTR(hotspot_pm, 0666, hotspot_pm_show, hotspot_pm_store);

static struct kobj_attribute dbm_attribute =
	__ATTR(dbm, 0644, dbm_show, dbm_store);

static struct attribute *my_attributes[] = {
	&max_pm_attribute.attr,
	&wake_pm_attribute.attr,
	&hotspot_pm_attribute.attr,
	&dbm_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = my_attributes,
};

static struct kobject *wifi_pm_kobj;

static int __init wifi_pm_init(void)
{
	int retval;

	wifi_pm_kobj = kobject_create_and_add("wifi_pm", kernel_kobj);
	if (!wifi_pm_kobj) {
		return -ENOMEM;
	}

	retval = sysfs_create_group(wifi_pm_kobj, &attr_group);
	if (retval)
		kobject_put(wifi_pm_kobj);

	return retval;
}
/* end sysfs interface */

static void __exit wifi_pm_exit(void)
{
	kobject_put(wifi_pm_kobj);
}

module_init(wifi_pm_init);
module_exit(wifi_pm_exit);

