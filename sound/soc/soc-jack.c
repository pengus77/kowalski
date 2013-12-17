/*
 * soc-jack.c  --  ALSA SoC jack handling
 *
 * Copyright 2008 Wolfson Microelectronics PLC.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <sound/jack.h>
#include <sound/soc.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <trace/events/asoc.h>

#if defined (CONFIG_MACH_STAR) 

#include <linux/wakelock.h>

#define HEADSET_CONNECTED 1
#define HEADSET_DISCONNECTED 0
#define HEADSET_NONE 2

#define INT_NONE 0
#define INT_ING 1
unsigned int int_status = INT_NONE;

unsigned int hook_status = HOOK_RELEASED;
headset_type_enum headset_type = STAR_NONE;
int hook_detection_time = 700; 

extern void star_headsetdet_bias(int bias);
extern void star_Mic_bias(int bias);

struct wake_lock headset_wake_lock;

unsigned int headset_status = HEADSET_NONE;

unsigned int headset_gpio_status = 0;
unsigned int hookkey_gpio_status = 0;
unsigned int headset_detecting = 0;

bool block_hook_int = false;
bool is_hook_test = false;

int type_detection_time = 700; 
int remove_detection_time = 60; 
#if !defined(STAR_COUNTRY_KR)
int hook_press_time = 100; 
int hook_release_time = 20; 
#else
int hook_press_time = 100; 
int hook_release_time = 20; 
#endif

int hook_press_delay = 500;

#endif

/**
 * snd_soc_jack_new - Create a new jack
 * @card:  ASoC card
 * @id:    an identifying string for this jack
 * @type:  a bitmask of enum snd_jack_type values that can be detected by
 *         this jack
 * @jack:  structure to use for the jack
 *
 * Creates a new jack object.
 *
 * Returns zero if successful, or a negative error code on failure.
 * On success jack will be initialised.
 */
int snd_soc_jack_new(struct snd_soc_codec *codec, const char *id, int type,
		     struct snd_soc_jack *jack)
{
	jack->codec = codec;
	INIT_LIST_HEAD(&jack->pins);
	INIT_LIST_HEAD(&jack->jack_zones);
	BLOCKING_INIT_NOTIFIER_HEAD(&jack->notifier);

	return snd_jack_new(codec->card->snd_card, id, type, &jack->jack);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_new);

/**
 * snd_soc_jack_report - Report the current status for a jack
 *
 * @jack:   the jack
 * @status: a bitmask of enum snd_jack_type values that are currently detected.
 * @mask:   a bitmask of enum snd_jack_type values that being reported.
 *
 * If configured using snd_soc_jack_add_pins() then the associated
 * DAPM pins will be enabled or disabled as appropriate and DAPM
 * synchronised.
 *
 * Note: This function uses mutexes and should be called from a
 * context which can sleep (such as a workqueue).
 */
void snd_soc_jack_report(struct snd_soc_jack *jack, int status, int mask)
{
	struct snd_soc_codec *codec;
	struct snd_soc_dapm_context *dapm;
	struct snd_soc_jack_pin *pin;
	int enable;
	int oldstatus;

	trace_snd_soc_jack_report(jack, mask, status);

	if (!jack)
		return;

	codec = jack->codec;
	dapm =  &codec->dapm;

	mutex_lock(&codec->mutex);

	oldstatus = jack->status;

	jack->status &= ~mask;
	jack->status |= status & mask;

	/* The DAPM sync is expensive enough to be worth skipping.
	 * However, empty mask means pin synchronization is desired. */
	if (mask && (jack->status == oldstatus))
		goto out;

	trace_snd_soc_jack_notify(jack, status);

	list_for_each_entry(pin, &jack->pins, list) {
		enable = pin->mask & jack->status;

		if (pin->invert)
			enable = !enable;

		if (enable)
			snd_soc_dapm_enable_pin(dapm, pin->pin);
		else
			snd_soc_dapm_disable_pin(dapm, pin->pin);
	}

	/* Report before the DAPM sync to help users updating micbias status */
	blocking_notifier_call_chain(&jack->notifier, status, jack);

	snd_soc_dapm_sync(dapm);

	snd_jack_report(jack->jack, jack->status);

out:
	mutex_unlock(&codec->mutex);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_report);

/**
 * snd_soc_jack_add_zones - Associate voltage zones with jack
 *
 * @jack:  ASoC jack
 * @count: Number of zones
 * @zone:  Array of zones
 *
 * After this function has been called the zones specified in the
 * array will be associated with the jack.
 */
int snd_soc_jack_add_zones(struct snd_soc_jack *jack, int count,
			  struct snd_soc_jack_zone *zones)
{
	int i;

	for (i = 0; i < count; i++) {
		INIT_LIST_HEAD(&zones[i].list);
		list_add(&(zones[i].list), &jack->jack_zones);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_add_zones);

/**
 * snd_soc_jack_get_type - Based on the mic bias value, this function returns
 * the type of jack from the zones delcared in the jack type
 *
 * @micbias_voltage:  mic bias voltage at adc channel when jack is plugged in
 *
 * Based on the mic bias value passed, this function helps identify
 * the type of jack from the already delcared jack zones
 */
int snd_soc_jack_get_type(struct snd_soc_jack *jack, int micbias_voltage)
{
	struct snd_soc_jack_zone *zone;

	list_for_each_entry(zone, &jack->jack_zones, list) {
		if (micbias_voltage >= zone->min_mv &&
			micbias_voltage < zone->max_mv)
				return zone->jack_type;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_get_type);

/**
 * snd_soc_jack_add_pins - Associate DAPM pins with an ASoC jack
 *
 * @jack:  ASoC jack
 * @count: Number of pins
 * @pins:  Array of pins
 *
 * After this function has been called the DAPM pins specified in the
 * pins array will have their status updated to reflect the current
 * state of the jack whenever the jack status is updated.
 */
int snd_soc_jack_add_pins(struct snd_soc_jack *jack, int count,
			  struct snd_soc_jack_pin *pins)
{
	int i;

	for (i = 0; i < count; i++) {
		if (!pins[i].pin) {
			printk(KERN_ERR "No name for pin %d\n", i);
			return -EINVAL;
		}
		if (!pins[i].mask) {
			printk(KERN_ERR "No mask for pin %d (%s)\n", i,
			       pins[i].pin);
			return -EINVAL;
		}

		INIT_LIST_HEAD(&pins[i].list);
		list_add(&(pins[i].list), &jack->pins);
	}

	/* Update to reflect the last reported status; canned jack
	 * implementations are likely to set their state before the
	 * card has an opportunity to associate pins.
	 */
	snd_soc_jack_report(jack, 0, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_add_pins);

/**
 * snd_soc_jack_notifier_register - Register a notifier for jack status
 *
 * @jack:  ASoC jack
 * @nb:    Notifier block to register
 *
 * Register for notification of the current status of the jack.  Note
 * that it is not possible to report additional jack events in the
 * callback from the notifier, this is intended to support
 * applications such as enabling electrical detection only when a
 * mechanical detection event has occurred.
 */
void snd_soc_jack_notifier_register(struct snd_soc_jack *jack,
				    struct notifier_block *nb)
{
	blocking_notifier_chain_register(&jack->notifier, nb);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_notifier_register);

/**
 * snd_soc_jack_notifier_unregister - Unregister a notifier for jack status
 *
 * @jack:  ASoC jack
 * @nb:    Notifier block to unregister
 *
 * Stop notifying for status changes.
 */
void snd_soc_jack_notifier_unregister(struct snd_soc_jack *jack,
				      struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&jack->notifier, nb);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_notifier_unregister);

#ifdef CONFIG_GPIOLIB
/* gpio detect */
static void snd_soc_jack_gpio_detect(struct snd_soc_jack_gpio *gpio)
{
	struct snd_soc_jack *jack = gpio->jack;
	int enable;
	int report;

	enable = gpio_get_value_cansleep(gpio->gpio);
	if (gpio->invert)
		enable = !enable;

	if (enable)
		report = gpio->report;
	else
		report = 0;

	if (gpio->jack_status_check)
		report = gpio->jack_status_check();

	snd_soc_jack_report(jack, report, gpio->report);
}

#ifndef CONFIG_MACH_STAR
/* irq handler for gpio pin */
static irqreturn_t gpio_handler(int irq, void *data)
{
	struct snd_soc_jack_gpio *gpio = data;
	struct device *dev = gpio->jack->codec->card->dev;

	trace_snd_soc_jack_irq(gpio->name);

	if (device_may_wakeup(dev))
		pm_wakeup_event(dev, gpio->debounce_time + 50);

	schedule_delayed_work(&gpio->work,
			      msecs_to_jiffies(gpio->debounce_time));

	return IRQ_HANDLED;
}
#endif

#if defined (CONFIG_MACH_STAR)
extern unsigned int max8907c_adc_read_hook_adc(void);

static void headset_det_work(struct work_struct *work)
{
	headset_gpio_status = gpio_get_value (headset_sw_data->gpio);

	if(headset_status == HEADSET_NONE && headset_gpio_status != HEADSET_CONNECTED) {
		return;
	}
	else {
		headset_status = headset_gpio_status;
	}

	if( (headset_status == HEADSET_CONNECTED) && (headset_type != STAR_NONE)){
		return;
	}

	if(headset_status == HEADSET_DISCONNECTED)
	{
		schedule_delayed_work(headset_sw_data->pdelayed_work, msecs_to_jiffies(remove_detection_time));

#if defined(STAR_COUNTRY_KR) && !defined(CONFIG_MACH_STAR_SKT_REV_A)
		headset_Mic_Bias(0);
#endif
	}
	else /* HEADSET_CONNECTED */
	{
		schedule_delayed_work(headset_sw_data->pdelayed_work, msecs_to_jiffies(type_detection_time));  // gpio_work()
		gpio_set_value(headset_sw_data->ear_mic, 1);  // Implement Hook Key.from STAR.
	}
}

void headset_enable()
{
	if(int_status == INT_ING) 
	{
		int_status = INT_NONE; 
		return;
	}
	schedule_work(&headset_sw_data->work);  // headset_det_work()
}
EXPORT_SYMBOL_GPL(headset_enable);

static void hook_det_work(struct work_struct *work)
{
	int hook_adc_value =0;

	if(is_hook_test == false)
		hook_adc_value = 20;
	else
		hook_adc_value = 65;

	hookkey_gpio_status = max8907c_adc_read_hook_adc();
	msleep(10);
	hookkey_gpio_status = max8907c_adc_read_hook_adc();

	if(headset_type != STAR_HEADSET || headset_status != HEADSET_CONNECTED)
		return;

	if(hook_status == HOOK_RELEASED){
		if(hookkey_gpio_status <= hook_adc_value){ 
			hook_status = HOOK_PRESSED; 
			input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 1);
			input_sync(headset_sw_data->ip_dev);
		}        
	}
	else{
		if(hookkey_gpio_status > hook_adc_value){ 
			hook_status = HOOK_RELEASED; 
			input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 0);
			input_sync(headset_sw_data->ip_dev);
		}
	}
}

static irqreturn_t headset_int_handler(int irq, void *dev_id)
{
	struct headset_switch_data *switch_data =
		(struct headset_switch_data *)dev_id;

	struct snd_soc_jack_gpio *gpio = switch_data->jack_gpio;
	struct device *dev = gpio->jack->codec->card->dev;

#define	HEADSET_DET_GPIO_NUM	51
	disable_irq_nosync(gpio_to_irq(HEADSET_DET_GPIO_NUM));
	if(headset_detecting)
		return IRQ_HANDLED;
#undef	HEADSET_DET_GPIO_NUM

	trace_snd_soc_jack_irq(gpio->name);

	if (device_may_wakeup(dev))
		pm_wakeup_event(dev, gpio->debounce_time + 50);

	headset_detecting = 1;

	int_status = INT_ING;

	headset_gpio_status = gpio_get_value (headset_sw_data->gpio);
	headset_status = headset_gpio_status;

	if(headset_status == HEADSET_DISCONNECTED)
	{
		schedule_work(headset_sw_data->pdelayed_work);
#if defined(STAR_COUNTRY_KR) && !defined(CONFIG_MACH_STAR_SKT_REV_A)  
		headset_Mic_Bias(0);
#endif
		gpio_set_value(headset_sw_data->ear_mic, 0);  //BCH_CHECK. Implement Hook Key. from STAR.
	}
	else
	{
		gpio_set_value(headset_sw_data->ear_mic, 1);  //BCH_CHECK. Implement Hook Key.from STAR.
		schedule_delayed_work(headset_sw_data->pdelayed_work,	msecs_to_jiffies(type_detection_time));	
		wake_lock_timeout(&headset_wake_lock, msecs_to_jiffies(type_detection_time + 50));
	}
	// Need to do Interrupt Done action

	return IRQ_HANDLED;
}

static irqreturn_t headset_hook_int_handler(int irq, void *dev_id)
{
	unsigned int gpio_hook = 1;	

	if( block_hook_int ){
		// hook interrupt done !!!
		return IRQ_HANDLED;
	}

	struct headset_switch_data	*switch_data = (struct headset_switch_data *)dev_id;

	if((headset_detecting == 1) || headset_type != STAR_HEADSET) 
		;
	else 
	{
		if(gpio_hook){
			schedule_delayed_work(&switch_data->hook_delayed_work,	msecs_to_jiffies(hook_release_time));
		}
		else{
			schedule_delayed_work(&switch_data->hook_delayed_work,	msecs_to_jiffies(hook_press_delay));
		}
	}

	return IRQ_HANDLED;
}
#endif

/* gpio work */
static void gpio_work(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio;
#if defined (CONFIG_MACH_STAR)
	unsigned int hook_value =0;
#endif

	gpio = container_of(work, struct snd_soc_jack_gpio, work.work);
	snd_soc_jack_gpio_detect(gpio);

#if defined (CONFIG_MACH_STAR)
	headset_status = headset_gpio_status;

	if( (headset_status == HEADSET_CONNECTED) && (headset_type == STAR_NONE))
	{
		headset_type = STAR_NONE;

		star_headsetdet_bias(1);

		hook_value = max8907c_adc_read_hook_adc();
		msleep(10);
		hook_value = max8907c_adc_read_hook_adc();

		if (hook_value > 350)
		{
			headset_type = STAR_HEADSET;
			hook_status = HOOK_RELEASED;
#if defined(CONFIG_MACH_STAR)
			gpio_set_value(headset_sw_data->ear_mic, 1);
#endif
		}
		else
		{
			headset_type = STAR_HEADPHONE;
			hook_status = HOOK_PRESSED;
#if defined(CONFIG_MACH_STAR)
			gpio_set_value(headset_sw_data->ear_mic, 0);
#endif
		}
	}
	else
	{
		headset_type = STAR_NONE;
		hook_status = HOOK_RELEASED; 
		headset_status = HEADSET_NONE;

#if defined(CONFIG_MACH_STAR)
		gpio_set_value(headset_sw_data->ear_mic, 0);
#endif

		input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 0);
		input_sync(headset_sw_data->ip_dev);
	}

	if(headset_type != STAR_HEADSET){
		block_hook_int = true;
		star_headsetdet_bias(0);
	}
	else
	{
		block_hook_int = false;
	}

	headset_detecting = 0;

	switch_set_state(&headset_sw_data->sdev, headset_type);

#define	HEADSET_DET_GPIO_NUM	51
	int_status = INT_NONE;
	enable_irq(gpio_to_irq(HEADSET_DET_GPIO_NUM));
#undef	HEADSET_DET_GPIO_NUM
#endif
}

/**
 * snd_soc_jack_add_gpios - Associate GPIO pins with an ASoC jack
 *
 * @jack:  ASoC jack
 * @count: number of pins
 * @gpios: array of gpio pins
 *
 * This function will request gpio, set data direction and request irq
 * for each gpio in the array.
 */
int snd_soc_jack_add_gpios(struct snd_soc_jack *jack, int count,
			struct snd_soc_jack_gpio *gpios)
{
	int i, ret;

	for (i = 0; i < count; i++) {
		if (!gpio_is_valid(gpios[i].gpio)) {
			printk(KERN_ERR "Invalid gpio %d\n",
				gpios[i].gpio);
			ret = -EINVAL;
			goto undo;
		}
		if (!gpios[i].name) {
			printk(KERN_ERR "No name for gpio %d\n",
				gpios[i].gpio);
			ret = -EINVAL;
			goto undo;
		}

#if defined (CONFIG_MACH_STAR)
		headset_sw_data->jack_gpio = &gpios[i];
#endif
        
		ret = gpio_request(gpios[i].gpio, gpios[i].name);
		if (ret)
			goto undo;

		ret = gpio_direction_input(gpios[i].gpio);
		if (ret)
			goto err;

#if defined (CONFIG_MACH_STAR)
		tegra_gpio_enable (gpios[i].gpio);

		INIT_WORK(&headset_sw_data->work, headset_det_work);
#endif
		INIT_DELAYED_WORK(&gpios[i].work, gpio_work);
		gpios[i].jack = jack;

#if defined (CONFIG_MACH_STAR)
		headset_sw_data->pdelayed_work = &gpios[i].work;
#endif

		ret = request_any_context_irq(gpio_to_irq(gpios[i].gpio),
#if defined (CONFIG_MACH_STAR)
					      headset_int_handler,
#else
					      gpio_handler,
#endif
					      IRQF_TRIGGER_RISING |
					      IRQF_TRIGGER_FALLING,
					      gpios[i].name,
#if defined (CONFIG_MACH_STAR)
					      headset_sw_data);
#else
					      &gpios[i]);
#endif

		if (ret < 0)
			goto err;

		if (gpios[i].wake) {
			ret = irq_set_irq_wake(gpio_to_irq(gpios[i].gpio), 1);
			if (ret != 0)
				printk(KERN_ERR
				  "Failed to mark GPIO %d as wake source: %d\n",
					gpios[i].gpio, ret);
		}

#if defined (CONFIG_MACH_STAR)
		ret = enable_irq_wake(gpio_to_irq(gpios[i].gpio));
		if (ret) {
			free_irq(gpio_to_irq(gpios[i].gpio), NULL);
			return ret;
		}

#if defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999)
		ret = gpio_request(headset_sw_data->ear_mic, "ear_mic");
		ret = gpio_direction_output(headset_sw_data->ear_mic, 0);
		if (ret)
			goto err;
		tegra_gpio_enable (headset_sw_data->ear_mic);
#endif

		ret = gpio_request(headset_sw_data->hook_gpio, "hook_det");
		ret = gpio_direction_input(headset_sw_data->hook_gpio);
		if (ret)
			goto err;
		tegra_gpio_enable (headset_sw_data->hook_gpio);		
		INIT_DELAYED_WORK(&headset_sw_data->hook_delayed_work, hook_det_work);

		headset_sw_data->hook_irq = gpio_to_irq(headset_sw_data->hook_gpio);

		ret = request_irq(headset_sw_data->hook_irq, headset_hook_int_handler,
				(IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING), "hook_det", headset_sw_data);
		if (ret)
			goto err;

		// please, make hook interrupt enable after detecting headset or headphone 
		ret = enable_irq_wake(headset_sw_data->hook_irq); 
		if (ret) {
			free_irq(headset_sw_data->hook_irq, NULL);
			return ret;
		}

		block_hook_int = true;
#endif

#ifdef CONFIG_GPIO_SYSFS
		/* Expose GPIO value over sysfs for diagnostic purposes */
		gpio_export(gpios[i].gpio, false);
#endif

		/* Update initial jack status */
		snd_soc_jack_gpio_detect(&gpios[i]);
	}

	return 0;

err:
	gpio_free(gpios[i].gpio);
#if defined (CONFIG_MACH_STAR)
	if(!headset_sw_data->hook_gpio)
        	gpio_free(headset_sw_data->hook_gpio);
#endif
undo:
	snd_soc_jack_free_gpios(jack, i, gpios);

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_add_gpios);

/**
 * snd_soc_jack_free_gpios - Release GPIO pins' resources of an ASoC jack
 *
 * @jack:  ASoC jack
 * @count: number of pins
 * @gpios: array of gpio pins
 *
 * Release gpio and irq resources for gpio pins associated with an ASoC jack.
 */
void snd_soc_jack_free_gpios(struct snd_soc_jack *jack, int count,
			struct snd_soc_jack_gpio *gpios)
{
	int i;

	for (i = 0; i < count; i++) {
#ifdef CONFIG_GPIO_SYSFS
		gpio_unexport(gpios[i].gpio);
#endif
		free_irq(gpio_to_irq(gpios[i].gpio), &gpios[i]);
		cancel_delayed_work_sync(&gpios[i].work);
		gpio_free(gpios[i].gpio);
		gpios[i].jack = NULL;
	}
}
EXPORT_SYMBOL_GPL(snd_soc_jack_free_gpios);
#endif	/* CONFIG_GPIOLIB */
