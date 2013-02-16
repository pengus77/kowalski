/*
 * Copyright (c) 2010 - 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach-tegra/gpio-names.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/pm.h>
#include <mach-tegra/board.h>
#include <lge/board-star.h>

#include <linux/LGE_touch_synaptics.h>
#include <linux/gpio_keys.h>

#ifdef CONFIG_ONETOUCHSCREEN_TEGRA_STAR
#include <linux/onetouch_synaptics.h>
#endif

bool is_star_touch_enable = false;


EXPORT_SYMBOL(is_star_touch_enable);

int touch_power_control(char* reg_id, bool on)
{
	static struct regulator *device_regulator = NULL;

	device_regulator = regulator_get(NULL, reg_id); 

	if (!device_regulator) {
		printk(KERN_ERR "[touch_power_control] Can't get reg_id (%s)!!\n", reg_id);		 	
		return -1;
	}

	if(on)
	{    
		regulator_enable(device_regulator);
	}
	else
	{            	
		regulator_disable(device_regulator);
	}
	return 0;
}

static struct star_synaptics_platform_data star_ts_data = {
	.gpio	    = TEGRA_GPIO_PX6,
	.power	    = &touch_power_control,
	.irqflags   = IRQF_TRIGGER_FALLING,
};

static const struct i2c_board_info star_i2c_touch_info[] = {
	{
		I2C_BOARD_INFO(LGE_TOUCH_NAME, LGE_TOUCH_ADDR),
		.irq		=	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PX6),
		.platform_data	=	&star_ts_data,
	},
};

#ifdef CONFIG_ONETOUCHSCREEN_TEGRA_STAR
static struct star_onetouch_synaptics_platform_data star_one_ts_data = {
	.gpio	    = TEGRA_GPIO_PJ6,
	.power	    = &touch_power_control,
	.irqflags   = IRQF_TRIGGER_LOW,
};

static const struct i2c_board_info star_i2c_onetouch_info[] = {
	{
		I2C_BOARD_INFO(ONETOUCH_SYNAPTICS_NAME, ONETOUCH_SYNAPTICS_ADDR),
		.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ6),
		.platform_data	= &star_one_ts_data,
	},
};
#endif

int __init star_touch_init(void)
{
#ifdef CONFIG_ONETOUCHSCREEN_TEGRA_STAR
	tegra_gpio_enable(TEGRA_GPIO_PJ6);
	i2c_register_board_info(0, star_i2c_onetouch_info, ARRAY_SIZE(star_i2c_onetouch_info));
#endif

	tegra_gpio_enable(TEGRA_GPIO_PX6);
	i2c_register_board_info(0, star_i2c_touch_info, ARRAY_SIZE(star_i2c_touch_info));

	return 0;
}


#ifdef CONFIG_STAR_TOUCH_LED
#include <linux/mfd/max8907c.h>
static struct max8907c_led_platform_data star_led_platform_data = {
	.default_trigger = "timer",
	.max_uA          = 27899,
};

static struct platform_device star_touch_led_device = {
	.name   =   "star_led",
	.id     =   -1,
	.dev	=	{
		.platform_data  = &star_led_platform_data,
	},
};

int __init star_touch_led_init(void)
{
	printk(KERN_INFO " %s : Touch LED init in mach dir \n", __func__);
	platform_device_register(&star_touch_led_device);
	return	0;
}
#endif
