/*
	Copyright 2021 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils.h"
#include <math.h>
#include "terminal.h"
#include "mc_interface.h"

// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// Phase filters
	palSetPadMode(PHASE_FILTER_GPIO, PHASE_FILTER_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	PHASE_FILTER_OFF();

	// AUX pin
	AUX_OFF();
	palSetPadMode(AUX_GPIO, AUX_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

#ifdef HW_USE_DAC
	// hw_setup_dac();
#else
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG); // ext1 / throttle
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG); // ext2 / regen
#endif

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG); // current 1
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG); // current 2
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG); // current 3
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG); // servo conn / ext3

	// palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG); // pwm out

	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);  // GDRV VSENSE
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);  // fet temp
	palSetPadMode(GPIOB, 5, PAL_MODE_INPUT_ANALOG);  // ext3

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG); // volt 1
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG); // volt 2
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG); // volt 3
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG); // vbus
	// palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG); // reverse
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG); // motor temp

#if defined(ENABLE_SHUTDOWN_SWITCH)
	static void terminal_shutdown_now(int argc, const char **argv);
	static void terminal_button_test(int argc, const char **argv);

	terminal_register_command_callback(
		"shutdown",
		"Shutdown VESC now.",
		0,
		terminal_shutdown_now);

	terminal_register_command_callback(
		"test_button",
		"Try sampling the shutdown button",
		0,
		terminal_button_test);
#endif
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels														// [vector number] signal name
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles); 	// [0] 	SENS1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, ADC_SampleTime_15Cycles);		// [3] 	CURR1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_15Cycles);		// [6]  ADC_EXT1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_15Cycles);		// [9]	TEMP_PCB
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_15Cycles);	    // [12] V_GATE_DRIVER 
	// ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 6, ADC_SampleTime_15Cycles);	// [15]

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);	// [1]  SENS2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, ADC_SampleTime_15Cycles);	// [4]  CURR2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 3, ADC_SampleTime_15Cycles);	// [7] 	ADC_EXT2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);	// [10]	TEMP_MOTOR
	ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 5, ADC_SampleTime_15Cycles);	// [13] ADC_EXT3
	// ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 6, ADC_SampleTime_15Cycles); // [16]

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);	// [2] SENS3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 2, ADC_SampleTime_15Cycles);	// [5] CURR3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 3, ADC_SampleTime_15Cycles);	// [8] AN_IN 
	ADC_RegularChannelConfig(ADC3, ADC_Channel_Vrefint, 4, ADC_SampleTime_15Cycles);// [11] vrefint
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 5, ADC_SampleTime_15Cycles);	// [14] unused
	// ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 6, ADC_SampleTime_15Cycles); // [17] unused

	// Injected channels - current sensors
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);  // CURR1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);  // CURR2
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);  // CURR3
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 2, ADC_SampleTime_15Cycles);  // CURR1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 2, ADC_SampleTime_15Cycles);  // CURR2
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_2, 2, ADC_SampleTime_15Cycles);  // CURR3
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_15Cycles);  // CURR1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 3, ADC_SampleTime_15Cycles);  // CURR2
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_2, 3, ADC_SampleTime_15Cycles);  // CURR3
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}

#if defined(ENABLE_SHUTDOWN_SWITCH)
bool hw_sample_shutdown_button(void) {
	chMtxLock(&shutdown_mutex);

	bt_diff = 0.0;

	for (int i = 0;i < 3;i++) {
		palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_INPUT_ANALOG);
		chThdSleep(5);
		float val1 = ADC_VOLTS(ADC_IND_SHUTDOWN);
		chThdSleepMilliseconds(1);
		float val2 = ADC_VOLTS(ADC_IND_SHUTDOWN);
		palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL);
		chThdSleepMilliseconds(1);

		bt_diff += (val1 - val2);
	}

	chMtxUnlock(&shutdown_mutex);

	return (bt_diff > 0.12);
}

static void terminal_shutdown_now(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	DISABLE_GATE();
	HW_SHUTDOWN_HOLD_OFF();
}

static void terminal_button_test(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	for (int i = 0;i < 40;i++) {
		commands_printf("BT: %d %.2f", HW_SAMPLE_SHUTDOWN(), (double)bt_diff);
		chThdSleepMilliseconds(100);
	}
}
#endif