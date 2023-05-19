/********************************************************************************
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 ********************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <string.h>

/********************************************************************************
 *
 ********************************************************************************/
enum led_id_t {
	LIGHTWELL_RED_LED,
	LIGHTWELL_GREEN_LED,
	LIGHTWELL_BLUE_LED
};

/********************************************************************************
 *
 ********************************************************************************/
/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 7

#define OPEN_VALVE		1
#define CLOSE_VALVE		0

#define HIGH			1
#define LOW				0

#define ON				1
#define OFF				0

#define MOTION_DETECTOR	13	/*  */

#define WATER_VALVE	 	16			/*  */
#define BUZZER	 		28 /* sig pin of the buzzer */
#define LIGHTWELL_RED 	29
#define LIGHTWELL_GREEN 30
#define LIGHTWELL_BLUE 	31

#define MAX_OUTPUTS		5
#define MAX_INPUTS		1

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* Option 1: by node label */
#define MY_GPIO0 DT_NODELABEL(gpio0)

/********************************************************************************
 * 
 ********************************************************************************/
// const struct device *gpio_dev;
const struct device *gpio_dev = DEVICE_DT_GET(MY_GPIO0);
//
struct k_timer my_timer;
// extern void my_expiry_function(struct k_timer *timer_id);
static struct gpio_callback motion_cb_data;

/********************************************************************************
 * 
 ********************************************************************************/
static uint8_t buzzer_state = OFF;
static uint32_t output_gpio[MAX_OUTPUTS] = { WATER_VALVE, BUZZER, LIGHTWELL_RED, 
											 LIGHTWELL_GREEN, LIGHTWELL_BLUE };
static uint32_t input_gpio[MAX_INPUTS] = { MOTION_DETECTOR };

/********************************************************************************
 * Play tone
 ********************************************************************************/
void beep_buzzer(int tone, int duration);
void motion_detected(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void my_expiry_function(struct k_timer *timer_id);
void blink(uint32_t sleep_ms, enum led_id_t id);
void blink0(void);
void blink1(void);
void blink2(void);
void configuer_all_input(void);
void configuer_all_output(void);

/********************************************************************************
 *
 ********************************************************************************/
K_THREAD_DEFINE(blink0_id, STACKSIZE, blink0, NULL, NULL, NULL,
				PRIORITY, 0, 0);
K_THREAD_DEFINE(blink1_id, STACKSIZE, blink1, NULL, NULL, NULL,
				PRIORITY, 0, 0);
K_THREAD_DEFINE(blink2_id, STACKSIZE, blink2, NULL, NULL, NULL,
				PRIORITY, 0, 0);


/********************************************************************************
 * 
 ********************************************************************************/
void blink(uint32_t sleep_ms, enum led_id_t id)
{
	int ret;
	
	switch(id) {
		case LIGHTWELL_RED_LED:
			ret = gpio_pin_toggle(gpio_dev, LIGHTWELL_RED);
			if (ret < 0) {
				return;
			}
			k_msleep(sleep_ms);
		break;

		case LIGHTWELL_GREEN_LED:
			ret = gpio_pin_toggle(gpio_dev, LIGHTWELL_GREEN);
			if (ret < 0) {
				return;
			}
			k_msleep(sleep_ms);
		break;

		case LIGHTWELL_BLUE_LED:
			ret = gpio_pin_toggle(gpio_dev, LIGHTWELL_BLUE);
			if (ret < 0)
			{
				return;
			}
			k_msleep(sleep_ms);
		break;

		default:
			break;
	}
}

/********************************************************************************
 * Play tone
 ********************************************************************************/
void beep_buzzer(int tone, int duration) {
    for (long i = 0; i < duration * 1000L; i += tone * 2) {
        gpio_pin_set(gpio_dev, BUZZER, HIGH);
        k_msleep(tone);
        gpio_pin_set(gpio_dev, BUZZER, LOW);
        k_msleep(tone);
    }
}

/********************************************************************************
 * 
 ********************************************************************************/
void blink0(void) {
	while(1) {
		blink(100, LIGHTWELL_RED_LED);
	}
}

/********************************************************************************
 *
 ********************************************************************************/
void blink1(void) {
	while(1) {
		blink(1000, LIGHTWELL_GREEN_LED);
	}
}

/********************************************************************************
 *
 ********************************************************************************/
void blink2(void) {
	while(1) {
		blink(5000, LIGHTWELL_BLUE_LED);
	}
}

/********************************************************************************
 * 
 ********************************************************************************/
void configuer_all_output(void) {
	int err;
	for(uint32_t i = 0; i < MAX_OUTPUTS; i++) {
		if (!device_is_ready(gpio_dev)) { 
			return;
		}

		err = gpio_pin_configure(gpio_dev, output_gpio[i], GPIO_OUTPUT_INACTIVE);
		if (err < 0) {
			return;
		}	
	}
}

/********************************************************************************
 * 
 ********************************************************************************/
void configuer_all_input(void) {
	int err;
	for(uint32_t i = 0; i < MAX_INPUTS; i++) {
		if (!device_is_ready(gpio_dev)) { 
			return;
		}

		err = gpio_pin_configure(gpio_dev, input_gpio[i], GPIO_INPUT | GPIO_PULL_DOWN);
		if (err < 0) {
			return;
		}	
	}
}

/********************************************************************************
 * Define the callback function
 ********************************************************************************/
void motion_detected(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	buzzer_state = ON;
	gpio_pin_set(gpio_dev, WATER_VALVE, OPEN_VALVE);
	/* start periodic timer that expires once every second */
	k_timer_start(&my_timer, K_SECONDS(10), K_NO_WAIT);
}

/********************************************************************************
 * Define a variable of type static struct gpio_callback
 ********************************************************************************/
void my_expiry_function(struct k_timer *timer_id) {
	buzzer_state = OFF;
	gpio_pin_set(gpio_dev, WATER_VALVE, CLOSE_VALVE);
}

/********************************************************************************
 *
 ********************************************************************************/
void main(void)
{
	int ret;

	k_msleep(SLEEP_TIME_MS * 10);
	printk("A Smart Water Tap Leakage Controller IoT Project/n/r");

	configuer_all_output();
	configuer_all_input();

	/* Configure the interrupt on the button's pin */
	ret = gpio_pin_interrupt_configure(gpio_dev, MOTION_DETECTOR, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		return;
	}

	/* Initialize the static struct gpio_callback variable */
	gpio_init_callback(&motion_cb_data, motion_detected, BIT(13));
	/* Add the callback function by calling gpio_add_callback() */
	gpio_add_callback(gpio_dev, &motion_cb_data);

	//
	k_timer_init(&my_timer, my_expiry_function, NULL);

	//
	while (1) {
		/*if(buzzer_state == ON) {
			beep_buzzer(3, 1);
			k_msleep(SLEEP_TIME_MS);
		}*/
		beep_buzzer(3, 1);
		k_msleep(SLEEP_TIME_MS);
	}
}

