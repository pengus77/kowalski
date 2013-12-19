/* Copyright (c) 2012, Will Tisdale <willtisdale@gmail.com>. All rights reserved.
 *
 * 2012 Enhanced by motley <motley.slate@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#if defined(CONFIG_SUSPEND) && defined(CONFIG_KOWALSKI_CPU_CONTROL)

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/earlysuspend.h>

#define CPUS_AVAILABLE		num_possible_cpus()

struct delayed_work hotplug_online_all_work;
struct delayed_work hotplug_offline_all_work;

extern bool auto_hotplug_enabled;
extern bool single_core_mode;

static void __cpuinit hotplug_online_all_work_fn(struct work_struct *work)
{
	int cpu;
	for_each_possible_cpu(cpu) {
		if (likely(!cpu_online(cpu))) {
			cpu_up(cpu);
		}
	}
}

static void __cpuinit hotplug_offline_all_work_fn(struct work_struct *work)
{
	int cpu;
	for_each_possible_cpu(cpu) {
		if (likely(cpu_online(cpu) && (cpu))) {
			cpu_down(cpu);
		}
	}
}

static void kowalski_hp_resume(struct early_suspend *handler){
	if (!auto_hotplug_enabled || single_core_mode)
		return;

	schedule_delayed_work_on(0, &hotplug_online_all_work, HZ);
}

static void kowalski_hp_suspend(struct early_suspend *handler) {
	if (!auto_hotplug_enabled || single_core_mode)
		return;

	if (num_online_cpus() > 1) {
		schedule_delayed_work_on(0, &hotplug_offline_all_work, HZ);
	}
}

static struct early_suspend kowalski_hp_suspend_handler = {
	.suspend = kowalski_hp_suspend,
	.resume = kowalski_hp_resume,
};

int __init auto_hotplug_init(void)
{
	pr_info("auto_hotplug: v0.220 by _thalamus\n");
	pr_info("auto_hotplug: rev 1 enhanced by motley\n");
	pr_info("auto_hotplug: rev 1.4 modified by pengus77\n");
	pr_info("auto_hotplug: %d CPUs detected\n", CPUS_AVAILABLE);

	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_online_all_work, hotplug_online_all_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_offline_all_work, hotplug_offline_all_work_fn);

	register_early_suspend(&kowalski_hp_suspend_handler);
}
late_initcall(auto_hotplug_init);

#endif /* CONFIG_SUSPEND */
