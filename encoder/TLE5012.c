#include "encoder/TLE5012.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "mc_interface.h"
#include "utils.h"
#include "spi_bb.h"
#include <math.h>

static TLE5012_config_t TLE5012_config_now = { 0 };

static float spi_error_rate = 0.0;
static float encoder_no_magnet_error_rate = 0.0;
static float encoder_no_magnet_error_cnt = 0.0;
static float last_enc_angle = 0.0;
static uint32_t spi_error_cnt = 0;
static uint32_t spi_val = 0;

encoder_ret_t TLE5012_init(TLE5012_config_t *TLE5012_config) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	TLE5012_config_now = *TLE5012_config;
	ssc_bb_init(&(TLE5012_config_now.sw_spi));

	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ((168000000 / 2
			/ TLE5012_config->refresh_rate_hz) - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(HW_ENC_TIM, &TIM_TimeBaseStructure);

	// Enable overflow interrupt
	TIM_ITConfig(HW_ENC_TIM, TIM_IT_Update, ENABLE);

	// Enable timer
	TIM_Cmd(HW_ENC_TIM, ENABLE);

	spi_error_rate = 0.0;
	encoder_no_magnet_error_rate = 0.0;

	nvicEnableVector(HW_ENC_TIM_ISR_CH, 6);

	TLE5012_config->is_init = 1;
	TLE5012_config_now = *TLE5012_config;

	return ENCODER_OK;
}

void TLE5012_deinit(void) {
	nvicDisableVector(HW_ENC_EXTI_CH);
	nvicDisableVector(HW_ENC_TIM_ISR_CH);

	TIM_DeInit(HW_ENC_TIM);

	ssc_bb_deinit(&(TLE5012_config_now.sw_spi));

	// palSetPadMode(TLE5012_config_now.sw_spi.miso_gpio,
	// 		TLE5012_config_now.sw_spi.miso_pin, PAL_MODE_INPUT_PULLUP);
	// palSetPadMode(TLE5012_config_now.sw_spi.sck_gpio,
	// 		TLE5012_config_now.sw_spi.sck_pin, PAL_MODE_INPUT_PULLUP);
	// palSetPadMode(TLE5012_config_now.sw_spi.nss_gpio,
	// 		TLE5012_config_now.sw_spi.nss_pin, PAL_MODE_INPUT_PULLUP);

	TLE5012_config_now.is_init = 0;
	last_enc_angle = 0.0;
	spi_error_rate = 0.0;
}

void TLE5012_routine(void) {
	uint16_t pos;
	uint16_t data;
	uint16_t safety_word;


	// steps
	// write 16bit command ()
	// delay by Twr (see datasheet)
	// read 16 bit angle (msb is rd_av, 1 for new angle available.)
	// read 16 bit safety word

	/*
	command word:
	bits
	[15] = rw, 1=read <- first bit transmitted
	[14..11] = lock, 0000
	[10] = Update-Register Access, 0: Access to current values, 1: values in buffer
	[9..4] = address, status=0x00, angle=0x02, speed=0x03
	[3..0] = 4-bit Number of Data Words (if bits set to 0000B, no safety word is provided)
	
	
	safety word:
	[15]:Indication of chip reset or watchdog overflow (resets after readout) via SSC
	[14]: System error
	[13]: Interface access error
	[12]: Invalid angle value (produce vesc fault if 1)
	[11..8]: Sensor number response indicator
	[7..0]: crc 
	*/

	ssc_bb_begin(&(TLE5012_config_now.sw_spi));
	ssc_bb_transfer_16(&(TLE5012_config_now.sw_spi), &data, 0, 1, true);
	ssc_bb_transfer_16(&(TLE5012_config_now.sw_spi), &data, 0, 1, false);
	ssc_bb_transfer_16(&(TLE5012_config_now.sw_spi), &safety_word, 0, 1, false);  // can this be disabled with command word?
	ssc_bb_end(&(TLE5012_config_now.sw_spi));

	// new_data_avail= data & 0x8000 // dont care, get angle anyways?
	pos = data & 0x7FFF;

	// if (spi_bb_check_parity(pos)) {
	// 	if (pos & TLE5012_NO_MAGNET_ERROR_MASK) {
	// 		++encoder_no_magnet_error_cnt;
	// 		UTILS_LP_FAST(encoder_no_magnet_error_rate, 1.0,
	// 				1. / TLE5012_config_now.refresh_rate_hz);
	// 	} else {
	// 		pos = pos >> 2;
	// 		last_enc_angle = ((float) pos * 360.0) / 16384.0;
	// 		UTILS_LP_FAST(spi_error_rate, 0.0,
	// 				1. / TLE5012_config_now.refresh_rate_hz);
	// 		UTILS_LP_FAST(encoder_no_magnet_error_rate, 0.0,
	// 				1. / TLE5012_config_now.refresh_rate_hz);
	// 	}
	// } else {
	// 	++spi_error_cnt;
	// 	UTILS_LP_FAST(spi_error_rate, 1.0,
	// 			1. / TLE5012_config_now.refresh_rate_hz);
	// }

}

float TLE5012_read_deg(void) {
	return last_enc_angle;
}

uint32_t TLE5012_spi_get_val(void) {
	return spi_val;
}

uint32_t TLE5012_spi_get_error_cnt(void) {
	return spi_error_cnt;
}

uint32_t TLE5012_get_no_magnet_error_cnt(void) {
	return encoder_no_magnet_error_cnt;
}

uint32_t TLE5012_get_no_magnet_error_rate(void) {
	return encoder_no_magnet_error_rate;
}
