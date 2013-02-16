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

#ifndef __MPUIRQ__
#define __MPUIRQ__

#include <linux/i2c-dev.h>
#include <linux/time.h>
#include <linux/ioctl.h>
#include "mldl_cfg.h"

#if defined (CONFIG_MACH_STAR)
#define MPUIRQ_ENABLE_DEBUG          (1)
#define MPUIRQ_GET_INTERRUPT_CNT     (2)
#define MPUIRQ_GET_IRQ_TIME          (3)
#define MPUIRQ_GET_LED_VALUE         (4)
#define MPUIRQ_SET_TIMEOUT           (5)
#define MPUIRQ_SET_ACCEL_INFO        (6)
#define MPUIRQ_SET_FREQUENCY_DIVIDER (7)
#define MPUIRQ_SET_WAKE_UP           (8)
#else
#define MPUIRQ_SET_TIMEOUT           _IOW(MPU_IOCTL, 0x40, unsigned long)
#define MPUIRQ_GET_INTERRUPT_CNT     _IOR(MPU_IOCTL, 0x41, unsigned long)
#define MPUIRQ_GET_IRQ_TIME          _IOR(MPU_IOCTL, 0x42, struct timeval)
#define MPUIRQ_SET_FREQUENCY_DIVIDER _IOW(MPU_IOCTL, 0x43, unsigned long)
#endif

void mpuirq_exit(void);
int mpuirq_init(struct i2c_client *mpu_client, struct mldl_cfg *mldl_cfg,
		unsigned long irq_flags);

#endif
