/*
	Copyright 2021 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_MOXIE_DRIVE_24_H_
#define HW_MOXIE_DRIVE_24_H_

#define HW_NAME				"Moxie_24"

// HW properties

// 25 mhz crystal
#define STM32_HSECLK                        25000000U
#define STM32_PLLM_VALUE                    25

#define HW_HAS_3_SHUNTS
// #define INVERTED_SHUNT_POLARITY  // not sure
// #define HW_HAS_PHASE_FILTERS

// throws out the standard vesc pinout. 
// adds specific pins for cruise/brake/reverse instead of repurposing.
// the interface pins are otherwise unaffected.
// #define ADDITIONAL_CONTROL_PINS 

// #define ENABLE_SHUTDOWN_SWITCH           


// todo:
// - fix adc mappings, mainly swap current and voltage inputs
// - add rev/brake sw pin mappings
// - remap servo
// - remap main serial
// - check ws2812 oputput pin.
// - fix current sensor scaling for 100a and 150a hall sensors.
// - add pwm control of 2 auxilary outputs. (8 bit, 0-255 is fine)
// - copy 12v intermediate bus sensing from axiom.
// - copy power switch code from hw 60
// - check can functionality (same pins as vesc6)

// Macros
#define LED_GREEN_GPIO			GPIOA
#define LED_GREEN_PIN			15
#define LED_RED_GPIO			GPIOD
#define LED_RED_PIN				2
// checked

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

#define PHASE_FILTER_GPIO		GPIOB
#define PHASE_FILTER_PIN		12
#define PHASE_FILTER_ON()		palSetPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
#define PHASE_FILTER_OFF()		palClearPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
// checked

#define AUX_GPIO				GPIOA
#define AUX_PIN					6
#define AUX2_GPIO				GPIOA
#define AUX2_PIN				7
#define AUX_ON()				palSetPad(AUX_GPIO, AUX_PIN)
#define AUX_OFF()				palClearPad(AUX_GPIO, AUX_PIN)
// checked

#if defined(ENABLE_SHUTDOWN_SWITCH)
// Shutdown pin
// sw_en = PB5 
// sw_sns = PC15
#define HW_SHUTDOWN_GPIO		GPIOB
#define HW_SHUTDOWN_PIN			5
#define HW_SHUTDOWN_HOLD_ON()	palSetPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
#define HW_SHUTDOWN_HOLD_OFF()	palClearPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
#define HW_SAMPLE_SHUTDOWN()	hw_sample_shutdown_button()

// Hold shutdown pin early to wake up on short pulses
#define HW_EARLY_INIT()			palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL); \
								HW_SHUTDOWN_HOLD_ON(); \
								palSetPadMode(GPIOD, 2, \
								PAL_MODE_OUTPUT_PUSHPULL | \
								PAL_STM32_OSPEED_HIGHEST); \
								CURRENT_FILTER_ON()
#endif

// pwm phase out pins checked

/*
non volt/curtrent adc channels
// vbus
// GDRV VSENSE
// fet temp
// motor temp

// ext1 / throttle
// ext2 / regen
// ext3 / cruise sw

*/

/*
 * ADC Vector moxie24
 *  all in#/names correct. apply to correct index
 *
 * DMA order|(adc)|adc mux in|signal name|verified (y)
 * 0  (1):	IN10	SENS1 y
 * 1  (2):	IN11	SENS2 y
 * 2  (3):	IN12	SENS3 y
 * 3  (1):	IN0 	CURR1 y
 * 4  (2):	IN1 	CURR2 y
 * 5  (3):	IN2 	CURR3 y
 *   
 * 6  (1):	IN4		ADC_EXT1
 * 7  (2):	IN5		ADC_EXT2
 * 8  (3):	IN13	AN_IN 
 * 9  (1):	IN9		TEMP_PCB
 * 10 (2):	IN15	TEMP_MOTOR
 * 11 (3):	Vrefint 
 * 12 (1):	IN8		V_GATE_DRIVER 
 * 13 (2):	IN14	ADC_EXT3, otherwise reverse button or servo 
 * 
 * 13 (X):	IN10	SENS1
 * 14 (X):	IN11	SENS2
 * 17 (X):  IN13	SENS3
 */


#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			5
#define HW_ADC_CHANNELS			(HW_ADC_NBR_CONV * 3)

// ADC Indexes
#define ADC_IND_SENS1			0
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			2
#define ADC_IND_CURR1			3
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			5

#define ADC_IND_VIN_SENS		8
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_EXT3			13
#define ADC_IND_SHUTDOWN		13 // ext3
#define ADC_IND_TEMP_MOS		9
#define ADC_IND_TEMP_MOTOR		10
#define ADC_IND_VREFINT			11
#define ADC_IND_VOUT_GATE_DRV	12


// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					47000.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		(13.33 / 1000)  // volts/amp, acs758, 150a bidirectional
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		1 // hall sensor
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))
// #define GET_INPUT_VOLTAGE()	 24


// NTC Termistors
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15)
// #define NTC_TEMP(adc_ind)		35 // testing

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)


// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE		0
#endif


// ADC GPIOs
// these pins are also capable of dac out.
#define HW_ADC_EXT_GPIO			GPIOA  // throttle
#define HW_ADC_EXT_PIN			4
#define HW_ADC_EXT2_GPIO		GPIOA  // regen
#define HW_ADC_EXT2_PIN			5
// fixme -- check definitaions. pins now correct, but adc broke.

// add this functionality
// #if defined(ADDITIONAL_CONTROL_PINS )
// gdrv_vsense =  PB0 // pull from axiom
// reverse_sw = PC4
// cruise = PC13

// UART Peripheral
#define HW_UART_DEV				SD1
#define HW_UART_GPIO_AF			GPIO_AF_USART1
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			6
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			7
// checked

// Permanent UART Peripheral (for NRF52)
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			SD4
#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
#define HW_UART_P_TX_PORT		GPIOC
#define HW_UART_P_TX_PIN		10
#define HW_UART_P_RX_PORT		GPIOC
#define HW_UART_P_RX_PIN		11
// checked

// NRF SWD
// for WT51822 module, use "BLE-Xtal:16M RX:1 TX:2 LED:3" firmware
#define NRF5x_SWDIO_GPIO		GPIOC
#define NRF5x_SWDIO_PIN			9
#define NRF5x_SWCLK_GPIO		GPIOB
#define NRF5x_SWCLK_PIN			2
// checked


// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM5
#define HW_ICU_TIMER			TIM5
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE)
#define HW_ICU_DEV				ICUD5
#define HW_ICU_CHANNEL			ICU_CHANNEL_2 // may have to add some extra definitions here, ch4
#define HW_ICU_GPIO_AF			GPIO_AF_TIM5
#define HW_ICU_GPIO				GPIOA
#define HW_ICU_PIN				3
// fixme - on PA3, tim5 ch4

// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11
// checked

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler
// checked

// SPI pins - not used.
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Override dead time. See the stm32f4 reference manual for calculating this value.
// measure this on moxxie drive!
#define HW_DEAD_TIME_NSEC		660.0

// Default setting overrides
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			11.0		// Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			70.0		// Maximum input voltage
#endif
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV					20000.0
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		100.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			60.0	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-50.0	// Input current limit in Amperes (Lower)
#endif

// Setting limits
#define HW_LIM_CURRENT			-100.0, 100.0
#define HW_LIM_CURRENT_IN		-100.0, 100.0
#define HW_LIM_CURRENT_ABS		0.0, 150.0
#define HW_LIM_VIN				11.0, 70.0
#define HW_LIM_ERPM				-100e3, 100e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 90.0

// HW-specific functions
// float hw100_500_get_temp(void);

#endif /* HW_100_500_H_ */
