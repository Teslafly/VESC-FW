/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Copyright 2022 Jakub Tomczak

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

#ifndef ENC_TS5700N8501_H_
#define ENC_TS5700N8501_H_

#include "datatypes.h"
#include "encoder/encoder_datatype.h"

encoder_ret_t enc_ts5700n8501_init(TS5700N8501_config_t *ts5700n8501_config);
void enc_ts5700n8501_deinit(void);

float enc_ts5700n8501_read_deg(void);

uint8_t* enc_ts5700n8501_get_raw_status(void);
int16_t enc_ts5700n8501_get_abm(void);
void enc_ts5700n8501_reset_errors(void);
void enc_ts5700n8501_reset_multiturn(void);

#endif /* ENC_TS5700N8501_H_ */
