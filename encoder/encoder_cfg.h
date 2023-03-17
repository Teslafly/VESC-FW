/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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

#ifndef ENCODER_CFG_H_
#define ENCODER_CFG_H_

#include "encoder_datatype.h"

#ifndef AS504x_NSS_PORT
#define AS504x_NSS_PORT		    HW_HALL_ENC_GPIO3
#endif
#ifndef AS504x_NSS_PIN
#define AS504x_NSS_PIN		    HW_HALL_ENC_PIN3
#endif
#ifndef AS504x_SCK_PORT
#define AS504x_SCK_PORT    	    HW_HALL_ENC_GPIO1
#endif
#ifndef AS504x_SCK_PIN
#define AS504x_SCK_PIN			HW_HALL_ENC_PIN1
#endif
#ifndef AS504x_MISO_PORT
#define AS504x_MISO_PORT		HW_HALL_ENC_GPIO2
#endif
#ifndef AS504x_MISO_PIN
#define AS504x_MISO_PIN		    HW_HALL_ENC_PIN2
#endif

#ifndef AD2S1205_NSS_PORT
#define AD2S1205_NSS_PORT	    HW_HALL_ENC_GPIO3
#endif
#ifndef AD2S1205_NSS_PIN
#define AD2S1205_NSS_PIN	    HW_HALL_ENC_PIN3
#endif
#ifndef AD2S1205_SCK_PORT
#define AD2S1205_SCK_PORT    	HW_HALL_ENC_GPIO1
#endif
#ifndef AD2S1205_SCK_PIN
#define AD2S1205_SCK_PIN	    HW_HALL_ENC_PIN1
#endif
#ifndef AD2S1205_MISO_PORT
#define AD2S1205_MISO_PORT		HW_HALL_ENC_GPIO2
#endif
#ifndef AD2S1205_MISO_PIN
#define AD2S1205_MISO_PIN		HW_HALL_ENC_PIN2
#endif


// Global encoder configurations
extern ABI_config_t encoder_cfg_ABI;
extern AD2S1205_config_t encoder_cfg_ad2s1205;
extern AS5x47U_config_t encoder_cfg_as5x47u;
extern AS504x_config_t encoder_cfg_as504x;
extern BISSC_config_t encoder_cfg_bissc;
extern MT6816_config_t encoder_cfg_mt6816;
extern ENCSINCOS_config_t encoder_cfg_sincos;
extern TLE5012_config_t encoder_cfg_tle5012;
extern TS5700N8501_config_t encoder_cfg_TS5700N8501;


#endif /* ENCODER_CFG_H_ */
