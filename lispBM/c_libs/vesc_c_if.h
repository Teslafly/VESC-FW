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

#ifndef VESC_C_IF_H
#define VESC_C_IF_H

#include <stdint.h>
#include <stdbool.h>

typedef bool (*load_extension_fptr)(char*,extension_fptr);
typedef void* lib_thread;

#ifdef IS_VESC_LIB
typedef uint32_t systime_t;

typedef struct {
	int id;
	systime_t rx_time;
	float rpm;
	float current;
	float duty;
} can_status_msg;

typedef struct {
	int id;
	systime_t rx_time;
	float amp_hours;
	float amp_hours_charged;
} can_status_msg_2;

typedef struct {
	int id;
	systime_t rx_time;
	float watt_hours;
	float watt_hours_charged;
} can_status_msg_3;

typedef struct {
	int id;
	systime_t rx_time;
	float temp_fet;
	float temp_motor;
	float current_in;
	float pid_pos_now;
} can_status_msg_4;

typedef struct {
	int id;
	systime_t rx_time;
	float v_in;
	int32_t tacho_value;
} can_status_msg_5;

typedef struct {
	int id;
	systime_t rx_time;
	float adc_1;
	float adc_2;
	float adc_3;
	float ppm;
} can_status_msg_6;

typedef enum {
	HW_TYPE_VESC = 0,
	HW_TYPE_VESC_BMS,
	HW_TYPE_CUSTOM_MODULE
} HW_TYPE;

typedef enum {
	FAULT_CODE_NONE = 0,
	FAULT_CODE_OVER_VOLTAGE,
	FAULT_CODE_UNDER_VOLTAGE,
	FAULT_CODE_DRV,
	FAULT_CODE_ABS_OVER_CURRENT,
	FAULT_CODE_OVER_TEMP_FET,
	FAULT_CODE_OVER_TEMP_MOTOR,
	FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,
	FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,
	FAULT_CODE_MCU_UNDER_VOLTAGE,
	FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,
	FAULT_CODE_ENCODER_SPI,
	FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,
	FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,
	FAULT_CODE_FLASH_CORRUPTION,
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,
	FAULT_CODE_UNBALANCED_CURRENTS,
	FAULT_CODE_BRK,
	FAULT_CODE_RESOLVER_LOT,
	FAULT_CODE_RESOLVER_DOS,
	FAULT_CODE_RESOLVER_LOS,
	FAULT_CODE_FLASH_CORRUPTION_APP_CFG,
	FAULT_CODE_FLASH_CORRUPTION_MC_CFG,
	FAULT_CODE_ENCODER_NO_MAGNET,
	FAULT_CODE_ENCODER_MAGNET_TOO_STRONG,
	FAULT_CODE_PHASE_FILTER,
} mc_fault_code;
#endif

typedef enum {
	VESC_PIN_COMM_RX = 0,
	VESC_PIN_COMM_TX,
	VESC_PIN_SWDIO,
	VESC_PIN_SWCLK,
	VESC_PIN_HALL1,
	VESC_PIN_HALL2,
	VESC_PIN_HALL3,
	VESC_PIN_ADC1,
	VESC_PIN_ADC2,
} VESC_PIN;

typedef enum {
	VESC_PIN_MODE_INPUT_NOPULL = 0,
	VESC_PIN_MODE_INPUT_PULL_UP,
	VESC_PIN_MODE_INPUT_PULL_DOWN,
	VESC_PIN_MODE_OUTPUT,
	VESC_PIN_MODE_OUTPUT_OPEN_DRAIN,
	VESC_PIN_MODE_OUTPUT_OPEN_DRAIN_PULL_UP,
	VESC_PIN_MODE_OUTPUT_OPEN_DRAIN_PULL_DOWN,
	VESC_PIN_MODE_ANALOG,
} VESC_PIN_MODE;

#ifndef PACKET_MAX_PL_LEN
#define PACKET_MAX_PL_LEN		512
#endif

#define PACKET_BUFFER_LEN		(PACKET_MAX_PL_LEN + 8)

// Types
typedef struct {
	void(*send_func)(unsigned char *data, unsigned int len);
	void(*process_func)(unsigned char *data, unsigned int len);
	unsigned int rx_read_ptr;
	unsigned int rx_write_ptr;
	int bytes_left;
	unsigned char rx_buffer[PACKET_BUFFER_LEN];
	unsigned char tx_buffer[PACKET_BUFFER_LEN];
} PACKET_STATE_t;

/*
 * Function pointer struct. Always add new function pointers to the end in order to not
 * break compatibility with old binaries.
 */
typedef struct {
	// LBM If
	load_extension_fptr lbm_add_extension;
	float (*lbm_dec_as_float)(lbm_value val);
	uint32_t (*lbm_dec_as_u32)(lbm_value val);
	int32_t (*lbm_dec_as_i32)(lbm_value val);
	lbm_value (*lbm_enc_float)(float f);
	lbm_value (*lbm_enc_u32)(uint32_t u);
	lbm_value (*lbm_enc_i32)(int32_t i);
	lbm_value (*lbm_cons)(lbm_value car, lbm_value cdr);
	lbm_value (*lbm_car)(lbm_value val);
	lbm_value (*lbm_cdr)(lbm_value val);
	bool (*lbm_is_array)(lbm_value val);
	int (*lbm_set_error_reason)(char *str);
	
	// Os
	void (*sleep_ms)(uint32_t ms);
	void (*sleep_us)(uint32_t us);
	int (*printf)(const char *str, ...);
	void* (*malloc)(size_t bytes);
	void (*free)(void *prt);
	lib_thread (*spawn)(void (*fun)(void *arg), size_t stack_size, char *name, void *arg);
	void (*request_terminate)(lib_thread thd);
	bool (*should_terminate)(void);
	void** (*get_arg)(uint32_t prog_addr);
	
	// ST IO
	void (*set_pad_mode)(void *gpio, uint32_t pin, uint32_t mode);
	void (*set_pad)(void *gpio, uint32_t pin);
	void (*clear_pad)(void *gpio, uint32_t pin);
	
	// Abstract IO
	bool (*io_set_mode)(VESC_PIN pin, VESC_PIN_MODE mode);
	bool (*io_write)(VESC_PIN pin, int state);
	bool (*io_read)(VESC_PIN pin);
	float (*io_read_analog)(VESC_PIN pin);
	bool (*io_get_st_pin)(VESC_PIN vesc_pin, void **gpio, uint32_t *pin);

	// CAN
	void (*can_set_sid_cb)(bool (*p_func)(uint32_t id, uint8_t *data, uint8_t len));
	void (*can_set_eid_cb)(bool (*p_func)(uint32_t id, uint8_t *data, uint8_t len));
	void (*can_transmit_sid)(uint32_t id, const uint8_t *data, uint8_t len);
	void (*can_transmit_eid)(uint32_t id, const uint8_t *data, uint8_t len);
	void (*can_send_buffer)(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send);
	void (*can_set_duty)(uint8_t controller_id, float duty);
	void (*can_set_current)(uint8_t controller_id, float current);
	void (*can_set_current_off_delay)(uint8_t controller_id, float current, float off_delay);
	void (*can_set_current_brake)(uint8_t controller_id, float current);
	void (*can_set_rpm)(uint8_t controller_id, float rpm);
	void (*can_set_pos)(uint8_t controller_id, float pos);
	void (*can_set_current_rel)(uint8_t controller_id, float current_rel);
	void (*can_set_current_rel_off_delay)(uint8_t controller_id, float current_rel, float off_delay);
	void (*can_set_current_brake_rel)(uint8_t controller_id, float current_rel);
	bool (*can_ping)(uint8_t controller_id, HW_TYPE *hw_type);
	can_status_msg* (*can_get_status_msg_index)(int index);
	can_status_msg* (*can_get_status_msg_id)(int id);
	can_status_msg_2* (*can_get_status_msg_2_index)(int index);
	can_status_msg_2* (*can_get_status_msg_2_id)(int id);
	can_status_msg_3* (*can_get_status_msg_3_index)(int index);
	can_status_msg_3* (*can_get_status_msg_3_id)(int id);
	can_status_msg_4* (*can_get_status_msg_4_index)(int index);
	can_status_msg_4* (*can_get_status_msg_4_id)(int id);
	can_status_msg_5* (*can_get_status_msg_5_index)(int index);
	can_status_msg_5* (*can_get_status_msg_5_id)(int id);
	can_status_msg_6* (*can_get_status_msg_6_index)(int index);
	can_status_msg_6* (*can_get_status_msg_6_id)(int id);

	// Motor Control
	int (*mc_motor_now)(void);
	void (*mc_select_motor_thread)(int motor);
	int (*mc_get_motor_thread)(void);
	bool (*mc_dccal_done)(void);
	void (*mc_set_pwm_callback)(void (*p_func)(void));
	mc_fault_code (*mc_get_fault)(void);
	const char* (*mc_fault_to_string)(mc_fault_code fault);
	void (*mc_set_duty)(float dutyCycle);
	void (*mc_set_duty_noramp)(float dutyCycle);
	void (*mc_set_pid_speed)(float rpm);
	void (*mc_set_pid_pos)(float pos);
	void (*mc_set_current)(float current);
	void (*mc_set_brake_current)(float current);
	void (*mc_set_current_rel)(float val);
	void (*mc_set_brake_current_rel)(float val);
	void (*mc_set_handbrake)(float current);
	void (*mc_set_handbrake_rel)(float val);
	int (*mc_set_tachometer_value)(int steps);
	void (*mc_release_motor)(void);
	bool (*mc_wait_for_motor_release)(float timeout);
	float (*mc_get_duty_cycle_now)(void);
	float (*mc_get_sampling_frequency_now)(void);
	float (*mc_get_rpm)(void);
	float (*mc_get_amp_hours)(bool reset);
	float (*mc_get_amp_hours_charged)(bool reset);
	float (*mc_get_watt_hours)(bool reset);
	float (*mc_get_watt_hours_charged)(bool reset);
	float (*mc_get_tot_current)(void);
	float (*mc_get_tot_current_filtered)(void);
	float (*mc_get_tot_current_directional)(void);
	float (*mc_get_tot_current_directional_filtered)(void);
	float (*mc_get_tot_current_in)(void);
	float (*mc_get_tot_current_in_filtered)(void);
	float (*mc_get_input_voltage_filtered)(void);
	int (*mc_get_tachometer_value)(bool reset);
	int (*mc_get_tachometer_abs_value)(bool reset);
	float (*mc_get_pid_pos_set)(void);
	float (*mc_get_pid_pos_now)(void);
	void (*mc_update_pid_pos_offset)(float angle_now, bool store);
	float (*mc_temp_fet_filtered)(void);
	float (*mc_temp_motor_filtered)(void);
	float (*mc_get_battery_level)(float *wh_left);
	float (*mc_get_speed)(void);
	float (*mc_get_distance)(void);
	float (*mc_get_distance_abs)(void);
	uint64_t (*mc_get_odometer)(void);
	void (*mc_set_odometer)(uint64_t new_odometer_meters);
	void (*mc_set_current_off_delay)(float delay_sec);
	float (*mc_stat_speed_avg)(void);
	float (*mc_stat_speed_max)(void);
	float (*mc_stat_power_avg)(void);
	float (*mc_stat_power_max)(void);
	float (*mc_stat_current_avg)(void);
	float (*mc_stat_current_max)(void);
	float (*mc_stat_temp_mosfet_avg)(void);
	float (*mc_stat_temp_mosfet_max)(void);
	float (*mc_stat_temp_motor_avg)(void);
	float (*mc_stat_temp_motor_max)(void);
	float (*mc_stat_count_time)(void);
	void (*mc_stat_reset)(void);

	// More (added here to not break binary compatibility)
	float (*system_time)(void);
	void (*commands_process_packet)(unsigned char *data, unsigned int len,
			void(*reply_func)(unsigned char *data, unsigned int len));

	// UART
	bool (*uart_start)(uint32_t baudrate, bool half_duplex);
	bool (*uart_write)(uint8_t *data, uint32_t size);
	int32_t (*uart_read)(void);

	// LBM
	char* (*lbm_dec_str)(lbm_value);
	int (*lbm_add_symbol_const)(char *, lbm_uint *);
	void (*lbm_block_ctx_from_extension)(void);
	bool (*lbm_unblock_ctx)(lbm_cid, lbm_value);
	lbm_cid (*lbm_get_current_cid)(void);

	// Packets
	void (*packet_init)(void (*s_func)(unsigned char *, unsigned int),
			 void (*p_func)(unsigned char *, unsigned int),
			 PACKET_STATE_t *);
	void (*packet_reset)(PACKET_STATE_t *);
	void (*packet_process_byte)(uint8_t, PACKET_STATE_t *);
	void (*packet_send_packet)(unsigned char *, unsigned int len, PACKET_STATE_t *);

} vesc_c_if;

typedef struct {
	void (*stop_fun)(void *arg);
	void *arg;
	uint32_t base_addr;
} lib_info;

// VESC-interface with function pointers
#define VESC_IF		((vesc_c_if*)(0x1000FC00))

// Put this at the beginning of your source file
#define HEADER		static volatile int __attribute__((__section__(".program_ptr"))) prog_ptr;

// Init function
#define INIT_FUN	bool __attribute__((__section__(".init_fun"))) init

// Put this at the start of the init function
#define INIT_START	(void)prog_ptr;

// Address of this program in memory
#define PROG_ADDR	((uint32_t)&prog_ptr)

// The argument that was set in the init function (same as the one you get in stop_fun)
#define ARG			(*VESC_IF->get_arg(PROG_ADDR))

#endif  // VESC_C_IF_H

