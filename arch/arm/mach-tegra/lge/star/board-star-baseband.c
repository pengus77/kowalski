/*
 * Copyright (C) 2011 NVIDIA Corporation
 *
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/tegra_caif.h>
#include <mach/tegra_usb_modem_power.h>
#include <lge/board-star-baseband.h>

/****************************************************************************************************
 *                                       SPI devices
 ****************************************************************************************************/
static struct spi_board_info __initdata spi_bus1_devices_info[] = {
	{
		.modalias = "ifxn721",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
		.max_speed_hz = 24000000,
		.irq = 0, // 277 ? GPIO_IRQ(TEGRA_GPIO_PO5),
		.platform_data = 0,
	},

};

static int spi_init(void)
{
	platform_device_register(&tegra_spi_device1);

	spi_register_board_info(spi_bus1_devices_info, ARRAY_SIZE(spi_bus1_devices_info));
	return 0;
}

int star_baseband_init(void)
{
	spi_init();

	return 0;
}
