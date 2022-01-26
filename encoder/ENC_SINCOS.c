
#include "encoder/ENC_SINCOS.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "utils.h"
#include <math.h>
#include "hw.h"

//TODO move defines to encoder_hwconf.h
#define SINCOS_SAMPLE_RATE_HZ       20000
#define SINCOS_MIN_AMPLITUDE        1.0         // sqrt(sin^2 + cos^2) has to be larger than this
#define SINCOS_MAX_AMPLITUDE        1.65        // sqrt(sin^2 + cos^2) has to be smaller than this

#define HW_HAS_SIN_COS_ENCODER //todo delete later

ENCSINCOS_config_t enc_sincos_config_now = { 0 };

static uint32_t sincos_signal_below_min_error_cnt = 0;
static uint32_t sincos_signal_above_max_error_cnt = 0;
static float sincos_signal_low_error_rate = 0.0;
static float sincos_signal_above_max_error_rate = 0.0;

static float last_enc_angle = 0.0;

void ENC_SINCOS_deinit(void) {
	last_enc_angle = 0.0;

	sincos_signal_low_error_rate = 0.0;
	sincos_signal_above_max_error_rate = 0.0;
	enc_sincos_config_now.is_init = 0;
}

encoder_ret_t ENC_SINCOS_init(ENCSINCOS_config_t *enc_sincos_config) {
	//ADC inputs are already initialized in hw_init_gpio()

	enc_sincos_config_now = *enc_sincos_config;

	sincos_signal_below_min_error_cnt = 0;
	sincos_signal_above_max_error_cnt = 0;
	sincos_signal_low_error_rate = 0.0;
	sincos_signal_above_max_error_rate = 0.0;
	last_enc_angle = 0.0;

	// ADC measurements needs to be in sync with motor PWM
#ifdef HW_HAS_SIN_COS_ENCODER
	enc_sincos_config->is_init = 1;
	enc_sincos_config_now = *enc_sincos_config;
	return ENCODER_OK;
#else
	enc_sincos_config->is_init = 0;
	return ENCODER_ERROR;
#endif
}
float ENC_SINCOS_read_deg(void) {
#ifdef HW_HAS_SIN_COS_ENCODER
	float angle = 0.0;
	float sin = ENCODER_SIN_VOLTS() * enc_sincos_config_now.s_gain - enc_sincos_config_now.s_offset; // ENCODER_SIN_VOLTS changed to ENCODER_SIN_VOLTS() otherwise the macro cannot be found
	float cos = ENCODER_COS_VOLTS() * enc_sincos_config_now.c_gain - enc_sincos_config_now.c_offset;// ENCODER_COS_VOLTS changed to ENCODER_COS_VOLTS() otherwise the macro cannot be found

	float module = SQ(sin) + SQ(cos);

	if (module > SQ(SINCOS_MAX_AMPLITUDE) )	{
		// signals vector outside of the valid area. Increase error count and discard measurement
		++sincos_signal_above_max_error_cnt;
		UTILS_LP_FAST(sincos_signal_above_max_error_rate, 1.0, 1./SINCOS_SAMPLE_RATE_HZ);
		angle = last_enc_angle;
	} else {
		if (module < SQ(SINCOS_MIN_AMPLITUDE)) {
			++sincos_signal_below_min_error_cnt;
			UTILS_LP_FAST(sincos_signal_low_error_rate, 1.0, 1./SINCOS_SAMPLE_RATE_HZ);
			angle = last_enc_angle;
		} else {
			UTILS_LP_FAST(sincos_signal_above_max_error_rate, 0.0, 1./SINCOS_SAMPLE_RATE_HZ);
			UTILS_LP_FAST(sincos_signal_low_error_rate, 0.0, 1./SINCOS_SAMPLE_RATE_HZ);

			float angle_tmp = RAD2DEG_f(utils_fast_atan2(sin, cos));
			UTILS_LP_FAST(angle, angle_tmp, enc_sincos_config_now.filter_constant);
			last_enc_angle = angle;
		}
	}
#endif
	return last_enc_angle;
}

uint32_t ENC_SINCOS_get_signal_below_min_error_cnt(void) {
	return sincos_signal_below_min_error_cnt;
}

uint32_t ENC_SINCOS_get_signal_above_max_error_cnt(void) {
	return sincos_signal_above_max_error_cnt;
}

float ENC_SINCOS_get_signal_below_min_error_rate(void) {
	return sincos_signal_low_error_rate;
}

float ENC_SINCOS_get_signal_above_max_error_rate(void) {
	return sincos_signal_above_max_error_rate;
}
