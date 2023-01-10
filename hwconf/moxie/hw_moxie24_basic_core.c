/*
	Copyright 2012-2022 Benjamin Vedder	benjamin@vedder.se

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
#include "utils_math.h"
#include "terminal.h"
#include "commands.h"
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

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG); // current 1
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG); // current 2
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG); // current 3
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG); // servo conn
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG); // ext1 / throttle
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG); // ext2 / regen
	// palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG); // pwm out

	// palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);  // GDRV VSENSE
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);  // fet temp
	// palSetPadMode(GPIOB, 5, PAL_MODE_INPUT_ANALOG);  // ext3

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG); // volt 1
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG); // volt 3
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG); // volt 3
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG); // vbus
	// palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG); // reverse
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG); // motor temp
}

void hw_setup_adc_channels(void) {
	uint8_t t_samp = ADC_SampleTime_15Cycles;

	// adc's are triggeres by mcpwm timer and run through list of regular
	// conversions and store the results to the ADC_Value[] array using DMA
	// position in this array is determined by adc number + sample order (zero indexed)
	// you must make sure that the adc channel io pin is available on that ADC

	// 1st conversion (by all adc's at once)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, t_samp);   // [0] 	PH CURR1
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, t_samp);	// [1] 	PH CURR2
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, t_samp);	// [2] 	PH CURR3

	// 2nd conversion
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, t_samp); 	// [3]	PH SENS1
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 2, t_samp); 	// [4]	PH SENS2
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, t_samp); 	// [5]	PH SENS3

	// 3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, t_samp);   // [6]
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, t_samp);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, t_samp);

	// 4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, t_samp);  // [9]
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, t_samp);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, t_samp);

	// 5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, t_samp);// [12]
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, t_samp);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, t_samp);

	





	// Injected channels - current sensors
	// ONLY USED FOR BLDC MODE
	// 3 injected measurements in a row for averaging
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);  // CURR1
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 2, ADC_SampleTime_15Cycles);  // CURR1
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_15Cycles);  // CURR1

	ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);  // CURR2
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 2, ADC_SampleTime_15Cycles);  // CURR2
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 3, ADC_SampleTime_15Cycles);  // CURR2

	ADC_InjectedChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);  // CURR3
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_2, 2, ADC_SampleTime_15Cycles);  // CURR3
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