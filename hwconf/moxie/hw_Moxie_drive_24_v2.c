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

//private functions
// static void terminal_cmd_doublepulse(int argc, const char** argv);

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

	#ifdef HW_USE_BRK
		// BRK Fault pin
		palSetPadMode(BRK_GPIO, BRK_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1));
	#endif

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

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG); // current 1
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG); // current 2
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG); // current 3
	// palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG); // servo conn
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG); // ext1 / throttle
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG); // ext2 / regen
	// palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG); // pwm out 1
	// palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_ANALOG); // pwm out 2

	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);  // GDRV VSENSE
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);  // fet temp

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG); // volt 1
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG); // volt 2
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG); // volt 3
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG); // vbus
	// palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG); // reverse
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG); // motor temp

// #if defined(ENABLE_SHUTDOWN_SWITCH)
// 	static void terminal_shutdown_now(int argc, const char **argv);
// 	static void terminal_button_test(int argc, const char **argv);

// 	terminal_register_command_callback(
// 		"shutdown",
// 		"Shutdown VESC now.",
// 		0,
// 		terminal_shutdown_now);

// 	terminal_register_command_callback(
// 		"test_button",
// 		"Try sampling the shutdown button",
// 		0,
// 		terminal_button_test);
// #endif

// #ifndef HW_HAS_DUAL_MOTORS
// 	// from hw_gesc.c
// 	//register terminal callbacks
// 	//double pulse not possible with dual motor setup
// 	terminal_register_command_callback(
// 		"double_pulse",
// 		"Start a double pulse test",
// 		0,
// 		terminal_cmd_doublepulse);
// #endif
}

void hw_setup_adc_channels(void) {
	// regular channel measurements are triggred mid pwm using scan mode + dma in foc mode.
	// order: phase currents -> phase voltages -> other

	// adc's are triggeres by mcpwm timer and run through list of regular
	// conversions and store the results to the ADC_Value[] array using DMA.
	// Position in this array is determined by adc number + sample order (zero indexed)
	// you must make sure that the adc channel i/o pin is available on that ADC

	// 1st conversion (by all 3 adc's at once)									// [vector number] signal name									
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, ADC_SampleTime_15Cycles);	// [0] CURR1
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, ADC_SampleTime_15Cycles);	// [1] CURR2
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 2, ADC_SampleTime_15Cycles);	// [2] CURR3
	// 2nd conversion
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles); // [3] VSENS1
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);	// [4] VSENS2
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);	// [5] VSENS3
	// 3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_15Cycles);	// [6] ADC_EXT1
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 3, ADC_SampleTime_15Cycles);	// [7] ADC_EXT2
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 3, ADC_SampleTime_15Cycles);	// [8] AN_IN
	// 4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_15Cycles);	// [9] TEMP_PCB
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);	// [10]	TEMP_MOTOR
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 4, ADC_SampleTime_15Cycles);  // [11] Servo pin
	// 5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_15Cycles);	// [12] V_GATE_DRIVER
	ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 5, ADC_SampleTime_15Cycles);	// [13] ADC_EXT3 (reverse sw)
	ADC_RegularChannelConfig(ADC3, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles); // [14] vrefint

	// Injected channels - current sensors only
	// ONLY USED FOR BLDC MODE
	// 3 injected measurements in a row for averaging. averaging does nothing for foc mode.
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

/*
static void terminal_cmd_powerstage_selftest_wo_motor(int argc, const char** argv){
	// must be run with motor unconnected. one of the first tests to run after powering on a new vesc hardware.
	// toggles the power stage one fet at a time to test voltage sensors
	// makes sure that:
	// - current sensors have valid offset.
	// vin, phase voltage, and current sensors are stable staticly. do not change more than 1-2%
	// measure mcu vin. is it within 5% of V_REG define? what is 3.0v vs 3.3v %?
	// - floating phases are not tied to vbat (or gnd?) aka are between 1 and 5 volts ish.
	// - phases can go low
	// - voltage sensed when phase is low is very close to 0v. (compare floating with low on)
	// - make sure other phases do not change state when toggle 1 phase.
	// then:
	// - phases can go high
	// - make sure other phases do not change state when toggle 1 phase.
	// - phase voltage sensor corrisponsds to correct phase
	// - sensed phase voltage is within x % of vbat voltage (calculate and retturn percentage)


	// - apply 50% duty cycle at default freq to make sure it doesnt blow up? or is that a job for foc_duty command?

    // can i test low side fets without the phases being connected together?
	// yes. probably. phases tend to float a few v above gnd beacuse of the bootstrap when not actively pulled down.
	// can probably measure that voltage.

}


static void terminal_cmd_powerstage_selftest_w_motor(int argc, const char** argv){
	// must be ran with motor connected
	// applies low duty cycle to motor one phase at a time to test current sensors.
	// 2 phases grounded, 1 phase 1-5% duty cycle. 2 low phase currents should be within 10?%
	// if phase current sense, phase 3 current should be -1*(phase1+phase2) (within 10%?)
	// just print accuracy precentages.
	// sequence:
	// - zero out current sensors (calcculat offset)
	// - watch current senssors for a few seconds. make sure arent unstable at 0 current (stay within 1% of zero?)
	// - 1 phase low. make surte all phases read less than 1% of vin
	// 	  - repeat for all phases.
	// - 1 phase high. make sure all phases measure high within 5% of vbatt. 
	//    - repeat for all phases.
	// - all phasess low
	// apply 1% duty cycle to phase for 1 second. get multiple current measurements.
	// check current measurements? are they insane? (should be less than 10% of abs max current)
	// compare current measurements. are low sides roughly equal?
	// are low side currents the right polarity (negative)
	// if phase sensors, is high side positive polarity and roughly equal to 2 low side phases?
	// are low sides on right phases. and high side on right phase?
	// repeat so each phase has been the high side phase.

// }
*/

/*
// from hw_gesc.c
// double puulse test for hardware verification
static void terminal_cmd_doublepulse(int argc, const char** argv)
{
	(void)argc;
	(void)argv;

	int preface, pulse1, breaktime, pulse2;
	int utick;
	int deadtime = -1;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	if (argc < 5) {
		commands_printf("Usage: double_pulse <preface> <pulse1> <break> <pulse2> [deadtime]");
		commands_printf("   preface: idle time in  µs");
		commands_printf("    pulse1: high time of pulse 1 in µs");
		commands_printf("     break: break between pulses in µs\n");
		commands_printf("    pulse2: high time of pulse 2 in µs");
		commands_printf("  deadtime: overwrite deadtime, in ns");
		return;
	}
	sscanf(argv[1], "%d", &preface);
	sscanf(argv[2], "%d", &pulse1);
	sscanf(argv[3], "%d", &breaktime);
	sscanf(argv[4], "%d", &pulse2);
	if (argc == 6) {
		sscanf(argv[5], "%d", &deadtime);
	}
	timeout_configure_IWDT_slowest();

	utick = (int)(SYSTEM_CORE_CLOCK / 1000000);
	mcpwm_deinit();
	mcpwm_foc_deinit();
	gpdrive_deinit();

	TIM_Cmd(TIM1, DISABLE);
	TIM_Cmd(TIM4, DISABLE);
	//TIM4 als Trigger Timer
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / 20000);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
	TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Enable);
	TIM4->CNT = 0;

	// TIM1
	// TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (preface + pulse1) * utick;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = preface * utick;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM2);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_Inactive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_Inactive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);


	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	if (deadtime < 0) {
		TIM_BDTRInitStructure.TIM_DeadTime = conf_general_calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	} else {
		TIM_BDTRInitStructure.TIM_DeadTime = conf_general_calculate_deadtime(deadtime, SYSTEM_CORE_CLOCK);
	}
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM1->CNT = 0;
	TIM1->EGR = TIM_EGR_UG;

	TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Trigger);
	TIM_SelectInputTrigger(TIM1, TIM_TS_ITR3);
	TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Single);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
	//Timer 4 triggert Timer 1
	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM4, DISABLE);
	TIM1->ARR = (breaktime + pulse2) * utick;
	TIM1->CCR1 = breaktime * utick;
	while (TIM1->CNT != 0);
	TIM_Cmd(TIM4, ENABLE);

	chThdSleepMilliseconds(1);
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	mc_configuration* mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();

	switch (mcconf->motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_init(mcconf);
		break;

	case MOTOR_TYPE_FOC:
		mcpwm_foc_init(mcconf, mcconf);
		break;

	case MOTOR_TYPE_GPD:
		gpdrive_init(mcconf);
		break;

	default:
		break;
	}
	commands_printf("Done");
	return;
}
*/