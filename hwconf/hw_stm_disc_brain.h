/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

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

#ifndef stm_disc_brain_H_
#define stm_disc_brain_H_


#define HW_NAME					"stm_disc_brain"


// HW properties
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS
//#define HW_HAS_PHASE_FILTERS
//#define INVERTED_SHUNT_POLARITY


// LD4: green LED is a user LED connected to the I/O PD12 
// LD5: red LED is a user LED connected to the I/O PD14
// LD6: blue LED is a user LED connected to the I/O PD15


// Macros
#define LED_GREEN_GPIO			GPIOD
#define LED_GREEN_PIN			12
#define LED_RED_GPIO			GPIOD
#define LED_RED_PIN				14

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

#define GATE_ENABLE_GPIO		GPIOB
#define GATE_ENABLE_PIN			7
#define ENABLE_GATE()			palClearPad(GATE_ENABLE_GPIO, GATE_ENABLE_PIN) // hw will explode (shootthrough) if you enable gate before configuring timer 1 pwm pins.
#define DISABLE_GATE()			palSetPad(GATE_ENABLE_GPIO, GATE_ENABLE_PIN) // need to add hw pullup

// pwm pins define, dont change unless you know what you're doing
#define PWM_H1_GPIO             GPIOA
#define PWM_H1_PIN              8



// #ifdef HW75_300_REV_2
// #define PHASE_FILTER_GPIO		GPIOC
// #define PHASE_FILTER_PIN		9
// #else
// #define PHASE_FILTER_GPIO		GPIOC
// #define PHASE_FILTER_PIN		11
// #endif
// #define PHASE_FILTER_ON()		palSetPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
// #define PHASE_FILTER_OFF()		palClearPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)

// #define AUX_GPIO				GPIOC
// #define AUX_PIN					12
// #define AUX_ON()				palSetPad(AUX_GPIO, AUX_PIN)
// #define AUX_OFF()				palClearPad(AUX_GPIO, AUX_PIN)

// #define CURRENT_FILTER_ON()		palSetPad(GPIOD, 2)
// #define CURRENT_FILTER_OFF()	palClearPad(GPIOD, 2)

/*
 * ADC Vector old
 *
 * 0  (1):	IN0		SENS1
 * 1  (2):	IN1		SENS2
 * 2  (3):	IN2		SENS3
 * 3  (1):	IN10	CURR1
 * 4  (2):	IN11	CURR2
 * 5  (3):	IN12	CURR3
 * 
 * 6  (1):	IN5		ADC_EXT1
 * 7  (2):	IN6		ADC_EXT2
 * 8  (3):	IN3		TEMP_MOS
 * 9  (1):	IN14	TEMP_MOTOR
 * 10 (2):	IN15	ADC_EXT3
 * 11 (3):	IN13	AN_IN
 * 12 (1):	Vrefint
 * 13 (2):	IN0		SENS1
 * 14 (3):	IN1		SENS2
 * 15 (1):  IN8		TEMP_MOS_2
 * 16 (2):  IN9		TEMP_MOS_3
 * 17 (3):  IN3		SENS3
 */

 /*
 * ADC Vector new moxie
 *
 * 0  (1):	IN0		SENS1 y
 * 1  (2):	IN1		SENS2 y
 * 2  (3):	IN2		SENS3 y
 * 3  (1):	IN10	CURR1 y
 * 4  (2):	IN11	CURR2 y
 * 5  (3):	IN12	CURR3 y
 * 
 * 6  (1):	IN4	    ADC_EXT1 y
 * 7  (2):	IN5		ADC_EXT2 y
 * 8  (3):	IN3	    servo/ADC_EXT3 y
 * 9  (1):	IN15	TEMP_MOTOR y
 * 10 (2):	IN14	TEMP_MOS y
 * 11 (3):	IN13	AN_IN y
 * 12 (1):	Vrefint  y
 * 13 (2):	IN0		SENS1
 * 14 (3):	IN1		SENS2
n* 15 (1):  IN8		
n* 16 (2):  IN9		
n* 17 (3):  IN3		SENS3
 */

#define HW_ADC_CHANNELS			15 //18
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			5 //6

// ADC Indexes
#define ADC_IND_SENS1			0
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			2
#define ADC_IND_CURR1			3
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			5

#define ADC_IND_VIN_SENS		11  
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_TEMP_MOS		10
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			12

// ADC macros and settings
// 9.9v input = 0.63v out. , 15.7 conv factor,
// ((VIN_R1 + VIN_R2) / VIN_R2)) = 15.7
// r2 = 1000, r1=14700
// 10vin, 0.755 vout, 
// 20vin , 1.51vdiv = 13.25 (no jumper)
// 47.3vin, 2.40vout, div=19.7 (72v jumper)

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.2
#endif
#ifndef VIN_R1
#define VIN_R1					14700.0
#endif
#ifndef VIN_R2
#define VIN_R2					1000.0
#endif
// #ifndef CURRENT_AMP_GAIN
// #define CURRENT_AMP_GAIN		20.0
// #endif
// #ifndef CURRENT_SHUNT_RES
// #define CURRENT_SHUNT_RES		(0.0005 / 3.0)
// #endif
# define hall_current_gain         0.020 // v/a, acs756

//#undef FAC_CURRENT
#define FAC_CURRENT					((V_REG / 4095.0) / (hall_current_gain))

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * 19.7)
// #define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))
//#define GET_INPUT_VOLTAGE()		12

#define NTC_TEMP_MOS_BETA 3380.0   // 
// NTC Termistors
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / NTC_TEMP_MOS_BETA) + (1.0 / 298.15)) - 273.15)
//#define NTC_TEMP(adc_ind)		35 // testing

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		1
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		1
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE		1
#endif


// adc app gpio
#define HW_REVERSE_PORT         GPIOC
#define HW_REVERSE_PIN          13
#define HW_BRAKE_PORT           GPIOB
#define HW_BRAKE_PIN            5



// UART Peripheral (good for moxie drive)
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

#define permanent_nrf

#ifdef permanent_nrf
// Permanent UART Peripheral (for NRF51) (good for moxie drive)
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			SD4
#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
#define HW_UART_P_TX_PORT		GPIOC
#define HW_UART_P_TX_PIN		10
#define HW_UART_P_RX_PORT		GPIOC
#define HW_UART_P_RX_PIN		11
#endif

// ICU Peripheral for servo decoding (needs changing)
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

// I2C Peripheral  (test on moxie drive)
#define HW_I2C_DEV				I2CD1
#define HW_I2C_GPIO_AF			GPIO_AF_I2C1
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			6
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			7

// Hall/encoder pins  (same on moxie drive)
#define HW_HALL_ENC_GPIO1		GPIOB
#define HW_HALL_ENC_PIN1		3
#define HW_HALL_ENC_GPIO2		GPIOB
#define HW_HALL_ENC_PIN2		5
#define HW_HALL_ENC_GPIO3		GPIOB
#define HW_HALL_ENC_PIN3		4
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

// SPI pins  (check/change on moxie drive)
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

//#define AS5047_USE_HW_SPI_PINS

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
#define HW_DEAD_TIME_NSEC		1400.0

// Default setting overrides
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			72.0	// Maximum input voltage
#endif
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_SW
#define MCCONF_FOC_F_SW					20000.0
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		150	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			true	// Run control loop in both v0 and v7 (requires phase shunts)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			100	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-10.0	// Input current limit in Amperes (Lower)
#endif

// Setting limits
#define HW_LIM_CURRENT			-250.0, 250.0
#define HW_LIM_CURRENT_IN		-100.0, 100.0
#define HW_LIM_CURRENT_ABS		0.0, 1000.0
#define HW_LIM_VIN				11.0, 72.0
#define HW_LIM_ERPM				-100e3, 100e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
// #define HW_LIM_TEMP_FET			-40.0, 80.0

// HW-specific functions

//configure inverted phases:
// TIM_OCNPolarity_High = low  -> leg off, high -> leg on, (default)
// TIM_OCNPolarity_Low = high -> leg off, low  -> leg on (inverted output)

//#define inverted_top_fet_driver    // uncomment to invert top (vbat) side fet signal
#define inverted_bottom_fet_driver // uncomment to invert bottom(gnd) side fet signal

#endif /* HW_stmdiscovery_frankentroller */
