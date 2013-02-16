const struct tegra_init_gpio_info tegra_sleep_gpio_info_array[] = {
    /* tristate off */
    { 0xFF,         0, SFIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       PMC},   // PMC

    /* Keep gpio output level */
    { 'w'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_LOW,   LM1},   // WLAN_EN
    { 'z'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_LOW,   LSDI},  // BT_EN
    { 's'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_HIGH,  KBCB},  // CHG_EN_SET_N_AP20
    { 'v'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_LOW,   UAC},   // IFX_RESET_1.8V
    { 'v'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_LOW,   UAC},   // IFX_PWRON_1.8V
    
    /* All GPIO output pins should be defined here */
    { 't'-'a',      4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   	  DTA},   // 8MN_CAM_VCM_EN
    { 'd'-'a',      5, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   	  DTA},   // VT_CAM_PWDN 
    { 't'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       DTB},   // FLASH_LED_TOURCH
    { 't'-'a',      3, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       DTB},   // FLASH_LED_INH
    { 'z'-'a' + 2,  1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       DTE},   // VT_RESET_N 
    { 'z'-'a' + 2,  4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       DTE},   // FLASH_LED_EN 
    { 'd'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       DTE},   // 8M_RESET_N
    { 'u'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       GPU},   // AP20_UART_SW
    { 'u'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       GPU},   // MDM_UART_SW
    { 'u'-'a',      4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       GPU},   // VIBE_EN
#if defined(CONFIG_MACH_STAR_P990)
    { 'r'-'a',      7, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,      KBCB},  // IFX_VBUS_EN
#elif defined(CONFIG_MACH_STAR_SU660)
    { 'r'-'a',      7, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       KBCB},  // DMB_EN
#else
	#error
#endif
    { 'r'-'a',      3, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       KBCD},  // BL_DCDC_RST_N
    { 'r'-'a',      6, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       KBCD},  // CAM_SUBPM_EN
    { 'v'-'a',      7, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       LVP0},  // LCD_RESET_N
    { 'k'-'a',      5, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       SPDO},  // HDMI_REG_EN
    { 'x'-'a',      4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       SPID},  // BT_WAKEUP
    { 'o'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       UAB},   // IPC_MRDY
    { 'i'-'a',      7, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,      ATC},   // MUIC_SCL
    { 'k'-'a',      4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,      ATC},   // MUIC_SDA
    { 'k'-'a',      3, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,      ATC},   // WM_LDO_EN
    { 'g'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       ATC},   // WLAN_WAKEUP 
    { 'u'-'a',      3, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       GPU},   // USIF1_SW (LGP90)
    { 'r'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       KBCA},  // IFX1_AP20 (sleep_status) (LGP990)
    { 'j'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,      GMD},   // GPS_RESET_N (LGP90) dynamic
    { 'j'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       GMD},   // GPS_PWR_ON (LGP90) dynamic

    /* All wakeup pins should be defined here : gpio input enable */
    { 'o'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       UAB},   // IPC_SRDY
    { 'z'-'a' + 2,  5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       DTE},   // NC  +
    { 'a'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       DTE},   // PROXI_OUT(NC) +
    { 'c'-'a',      7, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       GMB},   // BT_HOST_WAKEUP
    { 's'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       KBCB},  // WLAN_HOST_WAKEUP
    { 's'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       KBCB},  // CHG_STATUS_N_AP20
    { 'v'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       UAC},   // MDM_RESET_FLAG +
    { 'v'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       UAC},   // AP_PWR_ON(powerkey)    
    { 'w'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       SPIH},  // BATT_LOW_INT
    { 'n'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       LSDA},  // HOOK_DET

    /* input pins */
    { 'u'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       GPU},   // HALL_INT
    { 'u'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       GPU},   // INT_N_MUIC
    { 'u'-'a',      6, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       GPU},   // VIBE_PMW(LGP990), IPC_SRDY1(P999BN)
    { 'r'-'a',      4, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       KBCD},  // COM_INT
#if defined(CONFIG_MACH_STAR_P990)
    { 'r'-'a',	    5, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,       KBCD},  // NC
#elif defined(CONFIG_MACH_STAR_SU660)
    { 'r'-'a',	    5, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,      KBCD},  // IFX_VBUS_EN
#else
#error
#endif
    { 'q'-'a',	    5, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,       KBCF},  // GYRO_INT_N
    { 'q'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       KBCF},  // CHG_PGB_N
    { 'o'-'a',      6, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       UAB},   // SPI2_MISO
    { 'o'-'a',      7, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       UAB},   // SPI2_CLK
    { 'k'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       ATC},   // THERMAL_IRQ
    { 'g'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_HIGH,      ATC},   // VOL_KEY_UP
    { 'g'-'a',      1, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_HIGH,      ATC},   // VOL_KEY_DOWN
    { 'g'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       ATC},   // EARJACK_SENSE
    { 'i'-'a',	    0, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,	  ATC},   // MOTION_INT
    { 'x'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       SPIE},  // TOUCH_MAKER_ID
    { 'x'-'a',	    6, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,	  SPIE},  // TOUCH_INT
    { 'r'-'a',      1, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       KBCA},  // IFX2_AP20 (LGP990)
    { 'r'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       KBCA},  // PROXI_OUT
#if defined(CONFIG_MACH_STAR_SU660)
    { 'j'-'a',	    6, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       IRRX},  // LOWER_TOUCH_INT/ (SU660)
#else
    { 'j'-'a',	    6, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,	  IRRX},  // NC
#endif

    { 'j'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       IRTX},  // LCD_MAKER_ID
    { 't'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       DTD},   // VT_PCLK
    { 't'-'a',      1, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       CSUS},  // VT_MCLK

    { 'w'-'a',	    4, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,	  CDEV1}, // AUDIO_MCLK
    { 'w'-'a',	    5, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,	  CDEV2}, // AUDIO_MCLK2
    { 'v'-'a',      6, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,       GPV},   // HomeKey (SU660)
    { 'n'-'a',	    0, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,	  DAP1},  // DAP1
    { 'n'-'a',	    1, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,	  DAP1},  // DAP1
    { 'n'-'a',	    2, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,	  DAP1},  // DAP1
    { 'n'-'a',	    3, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,	  DAP1},  // DAP1
    { 'a'-'a',	    2, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,	  DAP2},  // DAP2
    { 'a'-'a',	    3, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,	  DAP2},  // DAP2
    { 'a'-'a',	    4, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,	  DAP2},  // DAP2
    { 'a'-'a',	    5, GPIO_ENABLE, GPIO_INPUT,	    GPIO_SLEEP_LOW,	  DAP2},  // DAP2
};
