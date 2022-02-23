
#ifndef ENCODER_TLE5012_H_
#define ENCODER_TLE5012_H_

#include "datatypes.h"
#include "encoder/encoder_datatype.h"

void TLE5012_deinit(void);
encoder_ret_t TLE5012_init(TLE5012_config_t *TLE5012_config);

float TLE5012_read_deg(void);
void TLE5012_routine(void);

uint32_t TLE5012_spi_get_val(void);
uint32_t TLE5012_spi_get_error_cnt(void);
uint32_t TLE5012_get_no_magnet_error_cnt(void);
uint32_t TLE5012_get_no_magnet_error_rate(void);

#endif /* ENCODER_TLE5012_H_ */
