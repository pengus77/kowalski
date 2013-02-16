/*
 * Cosmo TI MUIC TS5USBA33402 driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * Author: Sookyoung Kim <sookyoung.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/kernel.h>		/* printk() */
#include <linux/init.h>			/* __init, __exit */
#include <linux/uaccess.h>		/* copy_from/to_user() */
#include <linux/interrupt.h>		/* request_irq() */
#include <linux/irq.h>			/* set_irq_type() */
#include <linux/types.h>		/* kernel data types */
#include <asm/system.h>
#include "../gpio-names.h" 
/*
 * kernel/arch/arm/include/asm/gpio.h includes kernel/arch/arm/plat-omap/include/mach/gpio.h which,
 * in turn, includes kernel/include/asm-generic/gpio.h.
 * <mach/gpio.h> declares gpio_get|set_value(), gpio_to_irq().
 * <asm-generic/gpio.h> declares struct gpio_chip, gpio_request(), gpio_free(), gpio_direction_input|output().
 * The actual descriptions are in kernel/drivers/gpio/gpiolib.c and kernel/arch/arm/plat-omap/gpio.c.
 */
#include <asm/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>	/* usleep() */
#include <linux/proc_fs.h>
#include <linux/workqueue.h>	/* INIT_WORK() */
#include <linux/wakelock.h>

#include <linux/muic.h>
#include <linux/lge_hw_rev.h>


#define GPIO_CP_USB_VBUS_EN             TEGRA_GPIO_PF1  // MDM_USB_VBUS_EN
#define GPIO_CP_UART_SW                 TEGRA_GPIO_PU2  // UART_SW

/* 
 * Initialize MUIC, i.e., the CONTROL_1,2 and SW_CONTROL registers.
 * 1) Prepare to sense INT_STAT and STATUS bits.
 * 2) Open MUIC paths. -> To keep the path from uboot setting, remove this stage.
 */ 
void muic_init_ts5usba33402(TYPE_RESET reset)
{
	printk(KERN_INFO "[MUIC] muic_init_ts5usba33402()\n");

	if (reset == RESET) {
		/* Clear default switch position (0x03=0x24) */
		muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ); 
	}

	/*
	 * Iniialize ts5usba33402 for detection of accessories
	 * Enable 200K pull-up and ADC (0x01=0x12)
	 * Enable interrupt and set AUD Click/Pop resistor (0x02=0x50)
	 */
	muic_i2c_write_byte(SW_CONTROL, 0x3F);
	muic_i2c_write_byte(CONTROL_1, ID_200 | SEMREN | CP_EN);
	muic_i2c_write_byte(CONTROL_2, INT_EN);

	muic_mdelay(300);

	return;
}
EXPORT_SYMBOL(muic_init_ts5usba33402);


void set_ts5usba33402_ap_uart_mode(void)
{ 
	/* Connects CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* Connects AP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_AP_UART);

	/* Enables 200K, Charger Pump, and ADC (0x01=0x13) */
	muic_i2c_write_byte(CONTROL_1, ID_200 | SEMREN | CP_EN);

	/* Enable USB Path (0x03=0x00) */
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);
}


void set_ts5usba33402_cp_usb_mode(void) 
{
	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* Connect CP USB to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_USB);

	/* Enable 200K, Charger Pump, and ADC (0x01=0x13) */
	muic_i2c_write_byte(CONTROL_1, ID_200 | SEMREN | CP_EN);

	/* Enable UART Path (0x03=0x09) */
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);
}

void set_ts5usba33402_cp_uart_mode(void)
{
	/* Connects CP UART signals to DP3T */
	usif_switch_ctrl(USIF_DP3T);

	/* Connects CP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_UART);
	/* Enables 200K, Charger Pump, and ADC (0x01=0x13)  */
	muic_i2c_write_byte(CONTROL_1, ID_200 | SEMREN | CP_EN);

	/* Enables UART Path (0x03=0x00) */
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);
}

void set_ts5usba33402_ap_usb_mode(void)
{
	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);
	/*
	 * AP USB does not pass through DP3T.
	 * Just connect AP UART to MUIC UART. 
	 */
	dp3t_switch_ctrl(DP3T_NC);

	muic_i2c_write_byte(CONTROL_1, ID_200 | SEMREN | CP_EN);
	// INT_EN, CP_AUD, CHG_TYP, USB_DET_DIS on.
	muic_i2c_write_byte(CONTROL_2, INT_EN | CP_AUD | CHG_TYPE);

	// Connect DP, DM to USB_DP, USB_DM
	muic_i2c_write_byte(SW_CONTROL, DP_USB | DM_USB);
	//muic_mode = MUIC_AP_USB;

}


void set_ts5usba33402_muic_mode(unsigned char int_stat_value)
{
	gpio_set_value(GPIO_CP_UART_SW, 0);
	gpio_set_value(GPIO_CP_USB_VBUS_EN, 0);
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
	muic_i2c_write_byte(CONTROL_1, ID_200 | SEMREN | CP_EN);

	muic_mdelay(10);

	if (int_stat_value & V_VBUS) {
		if ((int_stat_value & IDNO) == IDNO_0010 || 
				(int_stat_value & IDNO) == IDNO_1001 ||
				(int_stat_value & IDNO) == IDNO_1010) {
			set_ts5usba33402_cp_usb_mode();
			muic_mode = MUIC_CP_USB;
			charging_mode = CHARGING_FACTORY;
		} else if ((int_stat_value & IDNO) == IDNO_0100) {
			set_ts5usba33402_cp_uart_mode();
			muic_mode = MUIC_CP_UART;
			charging_mode = CHARGING_FACTORY;
#if defined(CONFIG_MHL_TX_SII9244)
		} else if ((int_stat_value & IDNO) == IDNO_0000) {
			muic_set_mhl_mode_detect();
			muic_mode = MUIC_MHL;
			charging_mode = CHARGING_USB;
#endif
		} else if (int_stat_value & CHGDET) {
			muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
			muic_mode = MUIC_LG_TA;
			charging_mode = CHARGING_LG_TA;
		} else {
			set_ts5usba33402_ap_usb_mode();
			muic_mode = MUIC_AP_USB;
			charging_mode = CHARGING_USB;
		}
	} else {
		if ((int_stat_value & IDNO) == IDNO_0010) {
			set_ts5usba33402_ap_uart_mode();
			muic_mode = MUIC_AP_UART;
			charging_mode = CHARGING_NONE;
		} else if ((int_stat_value & IDNO) == IDNO_0100) {
			set_ts5usba33402_cp_uart_mode();
			muic_mode = MUIC_CP_UART;
			charging_mode = CHARGING_NONE;
		} else {
			muic_mode = MUIC_UNKNOWN;
			charging_mode = CHARGING_NONE;
		}
	}
}

s32 muic_ts5usba33402_detect_accessory(s32 upon_irq)
{
	s32 ret = 0;

	u8 int_stat_value;

	/*
	 * Upon an MUIC IRQ (MUIC_INT_N falls),
	 * wait 70ms before reading INT_STAT and STATUS.
	 * After the reads, MUIC_INT_N returns to high
	 * (but the INT_STAT and STATUS contents will be held).
	 *
	 * Do this only if muic_ts5usba33402_detect_accessory() was called upon IRQ. 
	 */
	muic_mdelay(70);

	/* Read INT_STAT */
	ret = muic_i2c_read_byte(INT_STAT, &int_stat_value);
	printk(KERN_DEBUG "[MUIC] muic_ts5usba33402_detect_accessory, int_stat_value:0x%02x \n", int_stat_value);

	if (ret < 0) {
		printk(KERN_INFO "[MUIC] INT_STAT reading failed\n");
		muic_mode = MUIC_UNKNOWN;
		charging_mode = CHARGING_UNKNOWN;
		return ret;
	}

	/* Branch according to the previous muic_mode */
	switch (muic_mode) {

		/* 
		 * MUIC_UNKNOWN is reached in two cases both do not have nothing to do with IRQ.
		 * First, at the initialization time where the muic_mode is not available yet.
		 * Second, whenever the current muic_mode detection is failed.
		 */
		case MUIC_UNKNOWN :

			/*
			 * If the previous muic_mode was MUIC_NONE,
			 * the only possible condition for a MUIC IRQ is plugging in an accessory.
			 */
		case MUIC_NONE :
			set_ts5usba33402_muic_mode(int_stat_value);
			break;

			/* 
			 * If the previous muic_mode was MUIC_NA_TA, MUIC_LG_TA, MUIC_TA_1A, MUIC_INVALID_CHG,
			 * MUIC_AP_UART, MUIC_CP_UART, MUIC_AP_USB, MUIC_OTG, or MUIC_CP_USB,
			 * the only possible condition for a MUIC IRQ is plugging out the accessory.
			 * 
			 * In this case, initialize MUIC and wait an IRQ.
			 * We don't need to wait 250msec because this is not an erronous case
			 * (we need to reset the facility to set STATUS for an erronous case and
			 * have to wait 250msec) and, if this is not an erronous case, the facility
			 * was already initialized at the system booting.
			 */
		case MUIC_AP_UART :
		case MUIC_CP_UART :
			if ((int_stat_value & IDNO) == IDNO_1011) {	        
				muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
				muic_mode = MUIC_NONE;			
				charging_mode = CHARGING_NONE;
			}       

			break;

		case MUIC_NA_TA :
		case MUIC_LG_TA :
		case MUIC_TA_1A :
		case MUIC_INVALID_CHG :
			if (((int_stat_value & V_VBUS) == 0) || ((int_stat_value & CHGDET) == 0)) {	         
				muic_mode = MUIC_NONE;				
				charging_mode = CHARGING_NONE;
			}

			break;

		case MUIC_AP_USB :
		case MUIC_CP_USB :
			/* 
			 * Check if V_VBUS == 0 && IDNO == 0x1011 (open).
			 * If so, it's MUIC_NONE.
			 * Otherwise, it's an erronous situation. MUIC_UNKNOWN.
			 */

			if((int_stat_value & IDNO) == IDNO_0010)
			{
				muic_mode = MUIC_CP_USB;
			}
			else if ((int_stat_value & IDNO) == IDNO_0100)
			{
				muic_mode = MUIC_CP_UART;
			}
			else if ((int_stat_value & V_VBUS) == 0) {         
				/* USB Host Removed */
				muic_mode = MUIC_NONE;				
				charging_mode = CHARGING_NONE;
			}
			break;
#if defined(CONFIG_MHL_TX_SII9244)
		case MUIC_MHL :
			printk(KERN_WARNING "[MUIC] Detect step3  MHL \n");
			if ((int_stat_value & V_VBUS) == 0) {    
				MHL_On(0);
				muic_mode = MUIC_NONE;
				charging_mode = CHARGING_NONE;
			}
			break;
#endif

		default:
			printk(KERN_WARNING "[MUIC] Failed to detect an accessory. Try again!");
			muic_mode = MUIC_UNKNOWN;
			charging_mode = CHARGING_NONE;
			ret = -1;
			break;
	}	

	if(muic_mode == MUIC_UNKNOWN || muic_mode == MUIC_NONE){
		muic_init_ts5usba33402(RESET);
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		printk(KERN_INFO "[MUIC] charging_ic_deactive()\n");
	}

	printk(KERN_DEBUG "[MUIC] muic_ts5usba33402_detect_accessory, muic_mode = %s (%d) \n", muic_mode_str[muic_mode], muic_mode);
	return ret;
} 
