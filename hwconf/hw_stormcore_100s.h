/*
    Copyright 2019 Benjamin Vedder  benjamin@vedder.se

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

#ifndef HW_STORMCORE_100S_H_
#define HW_STORMCORE_100S_H_

#include "drv8323s.h"

#define HW_NAME							"STORMCORE_100S"

// HW properties
#define HW_HAS_DRV8323S
#define HW_HAS_3_SHUNTS

#if defined(HW_VER_IS_100S_V2)
#define SWITCH_LED_3_GPIO				GPIOC
#define SWITCH_LED_3_PIN				15
#define SWITCH_LED_2_GPIO				GPIOC
#define SWITCH_LED_2_PIN				14
#define SWITCH_LED_1_GPIO				GPIOC
#define SWITCH_LED_1_PIN				13

#define LED_PWM1_ON()			palClearPad(SWITCH_LED_1_GPIO,SWITCH_LED_1_PIN)
#define LED_PWM1_OFF()			palSetPad(SWITCH_LED_1_GPIO,SWITCH_LED_1_PIN)
#define LED_PWM2_ON()			palClearPad(SWITCH_LED_2_GPIO, SWITCH_LED_2_PIN)
#define LED_PWM2_OFF()			palSetPad(SWITCH_LED_2_GPIO, SWITCH_LED_2_PIN)
#define LED_PWM3_ON()			palClearPad(SWITCH_LED_3_GPIO, SWITCH_LED_3_PIN)
#define LED_PWM3_OFF()			palSetPad(SWITCH_LED_3_GPIO, SWITCH_LED_3_PIN)


//#define HW_HAS_PHASE_FILTERS
#define PHASE_FILTER_GPIO		GPIOA
#define PHASE_FILTER_PIN		15
#define PHASE_FILTER_ON()		palSetPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
#define PHASE_FILTER_OFF()		palClearPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)

// Shutdown pin
#define HW_SHUTDOWN_GPIO		GPIOB
#define HW_SHUTDOWN_PIN			2
#define HW_SWITCH_SENSE_GPIO		GPIOB
#define HW_SWITCH_SENSE_PIN			12
#define HW_SHUTDOWN_HOLD_ON()	palSetPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
#define HW_SHUTDOWN_HOLD_OFF()	palClearPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
#define HW_SAMPLE_SHUTDOWN()	palReadPad(HW_SWITCH_SENSE_GPIO, HW_SWITCH_SENSE_PIN)

// Hold shutdown pin early to wake up on short pulses
#define HW_EARLY_INIT()			palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL); \
								HW_SHUTDOWN_HOLD_ON(); \
								palSetPadMode(HW_SWITCH_SENSE_GPIO, HW_SWITCH_SENSE_PIN, \
								PAL_MODE_INPUT_PULLDOWN); \
								palSetPadMode(PHASE_FILTER_GPIO, PHASE_FILTER_PIN, \
								PAL_MODE_OUTPUT_PUSHPULL | \
								PAL_STM32_OSPEED_HIGHEST); \
								PHASE_FILTER_OFF()
#endif

#define DRV8323S_CUSTOM_SETTINGS()		drv8323s_set_current_amp_gain(CURRENT_AMP_GAIN); \
		drv8323s_write_reg(3,0x3af); \
		drv8323s_write_reg(4,0x7af);

// Macros
#define ENABLE_GATE()			palSetPad(GPIOB, 5)
#define DISABLE_GATE()			palClearPad(GPIOB, 5)

#define DCCAL_ON()				//drv8323s_dccal_on()
#define DCCAL_OFF()				//drv8323s_dccal_off()

#define IS_DRV_FAULT()			(!palReadPad(GPIOB, 7))

#define LED_GREEN_ON()			palSetPad(GPIOB, 0)
#define LED_GREEN_OFF()			palClearPad(GPIOB, 0)
#define LED_RED_ON()			palSetPad(GPIOB, 1)
#define LED_RED_OFF()			palClearPad(GPIOB, 1)

//#define PHASE_FILTER_GPIO       GPIOC
//#define PHASE_FILTER_PIN        13
//#define PHASE_FILTER_ON()       palSetPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
//#define PHASE_FILTER_OFF()      palClearPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)

/*
 * ADC Vector
 *
 * 0:   IN0     SENS1
 * 1:   IN1     SENS2
 * 2:   IN2     SENS3
 * 3:   IN10    CURR1
 * 4:   IN11    CURR2
 * 5:   IN12    CURR3
 * 6:   IN5     ADC_EXT1
 * 7:   IN6     ADC_EXT2
 * 8:   IN3     TEMP_PCB
 * 9:   IN14    TEMP_MOTOR
 * 10:  IN15    ADC_EXT3
 * 11:  IN13    AN_IN
 * 12:  Vrefint
 * 13:  IN0     SENS1
 * 14:  IN1     SENS2
 */

#define HW_ADC_CHANNELS			15
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			5

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
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			12

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					68000.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif

#if defined(HW_VER_IS_100S_V2)
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.0005
#endif
#else
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		10.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.001
#endif
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()     ((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_RES(adc_val)        (10000.0 / ((4095.0 / (float)adc_val) - 1.0)) //NTC is low side onb this hardware
#define NTC_TEMP(adc_ind)       (1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val)  (10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)    (1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)           ((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE     0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE     0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE     0
#endif

// Number of servo outputs
#define HW_SERVO_NUM            1

// UART Peripheral
#define HW_UART_DEV             SD3
#define HW_UART_GPIO_AF         GPIO_AF_USART3
#define HW_UART_TX_PORT         GPIOB
#define HW_UART_TX_PIN          10
#define HW_UART_RX_PORT         GPIOB
#define HW_UART_RX_PIN          11


// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_DEV              ICUD4
#define HW_ICU_CHANNEL          ICU_CHANNEL_1
#define HW_ICU_GPIO_AF          GPIO_AF_TIM4
#define HW_ICU_GPIO             GPIOB
#define HW_ICU_PIN              6

// I2C Peripheral
#define HW_I2C_DEV              I2CD2
#define HW_I2C_GPIO_AF          GPIO_AF_I2C2
#define HW_I2C_SCL_PORT         GPIOB
#define HW_I2C_SCL_PIN          10
#define HW_I2C_SDA_PORT         GPIOB
#define HW_I2C_SDA_PIN          11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1       GPIOC
#define HW_HALL_ENC_PIN1        6
#define HW_HALL_ENC_GPIO2       GPIOC
#define HW_HALL_ENC_PIN2        7
#define HW_HALL_ENC_GPIO3       GPIOC
#define HW_HALL_ENC_PIN3        8
#define HW_ENC_TIM              TIM3
#define HW_ENC_TIM_AF           GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC     EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC      EXTI_PinSource8
#define HW_ENC_EXTI_CH          EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE        EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC     EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH       TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC      TIM3_IRQHandler

// SPI pins
#define HW_SPI_DEV              SPID1
#define HW_SPI_GPIO_AF          GPIO_AF_SPI1
#define HW_SPI_PORT_NSS         GPIOA
#define HW_SPI_PIN_NSS          4
#define HW_SPI_PORT_SCK         GPIOA
#define HW_SPI_PIN_SCK          5
#define HW_SPI_PORT_MOSI        GPIOA
#define HW_SPI_PIN_MOSI         7
#define HW_SPI_PORT_MISO        GPIOA
#define HW_SPI_PIN_MISO         6

// SPI for DRV8323S
#define DRV8323S_MOSI_GPIO      GPIOC
#define DRV8323S_MOSI_PIN       12
#define DRV8323S_MISO_GPIO      GPIOC
#define DRV8323S_MISO_PIN       11
#define DRV8323S_SCK_GPIO       GPIOC
#define DRV8323S_SCK_PIN        10
#define DRV8323S_CS_GPIO        GPIOC
#define DRV8323S_CS_PIN         9

// LSM6DS3
#define LSM6DS3_SDA_GPIO		GPIOB
#define LSM6DS3_SDA_PIN			3
#define LSM6DS3_SCL_GPIO		GPIOB
#define LSM6DS3_SCL_PIN			4

// Measurement macros
#define ADC_V_L1                ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2                ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3                ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO              (ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()            palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()            palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()            palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Default setting overrides
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE               92.0    // Maximum input voltage
#endif
#ifndef MCCONF_L_CURRENT_MAX
#define MCCONF_L_CURRENT_MAX                60.0    // Current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_CURRENT_MIN
#define MCCONF_L_CURRENT_MIN                -60.0   // Current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX             60.0    // Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN             -60.0   // Input current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT            200.0   // The maximum absolute current above which a fault is generated
#endif

#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE       MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_L_DUTY_START
#define MCCONF_L_DUTY_START				0.9 // Start limiting current at this duty cycle
#endif

// Setting limits
#define HW_LIM_CURRENT          -180.0, 180.0
#define HW_LIM_CURRENT_IN       -100.0, 100.0
#define HW_LIM_CURRENT_ABS      0.0, 200.0
#define HW_LIM_VIN              6.0, 92.0
#define HW_LIM_ERPM             -200e3, 200e3
#define HW_LIM_DUTY_MIN         0.0, 0.1
#define HW_LIM_DUTY_MAX         0.0, 0.99
#define HW_LIM_TEMP_FET         -40.0, 110.0

#endif /* HW_HV_stormcore_H_ */
