// Copyright 2023, Collabora, Ltd.
// Copyright 2023, Jarett Millard
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PlayStation Sense controller prober and driver code.
 * @author Jarett Millard <jarett.millard@gmail.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_pssense
 */

#pragma once

#include "xrt/xrt_byte_order.h"

#include "math/m_api.h"


#pragma pack(push, 1)

const uint8_t INPUT_REPORT_ID = 0x31;
const uint8_t OUTPUT_REPORT_ID = 0x31;
const uint8_t OUTPUT_REPORT_TAG = 0x10;
const uint8_t CALIBRATION_DATA_FEATURE_REPORT_ID = 0x05;
const uint8_t CALIBRATION_DATA_PART_ID_1 = 0;
const uint8_t CALIBRATION_DATA_PART_ID_2 = 0x81;

const uint8_t INPUT_REPORT_CRC32_SEED = 0xa1;
const uint8_t OUTPUT_REPORT_CRC32_SEED = 0xa2;
const uint8_t FEATURE_REPORT_CRC32_SEED = 0xa3;

//! Gyro read value range is +-32768.
const double PSSENSE_GYRO_SCALE_DEG = 180.0 / 1024;
//! Accelerometer read value range is +-32768 and covers +-8 g.
const double PSSENSE_ACCEL_SCALE = MATH_GRAVITY_M_S2 / 4096;

const uint8_t CHARGE_STATE_DISCHARGING = 0x00;
const uint8_t CHARGE_STATE_CHARGING = 0x01;
const uint8_t CHARGE_STATE_FULL = 0x02;
const uint8_t CHARGE_STATE_ABNORMAL_VOLTAGE = 0x0A;
const uint8_t CHARGE_STATE_ABNORMAL_TEMP = 0x0B;
const uint8_t CHARGE_STATE_CHARGING_ERROR = 0x0F;

#define INPUT_REPORT_LENGTH 78
/*!
 * HID input report data packet.
 */
struct pssense_input_report
{
	uint8_t report_id;
	uint8_t bt_header;
	uint8_t thumbstick_x;
	uint8_t thumbstick_y;
	uint8_t trigger_value;
	uint8_t trigger_proximity;
	uint8_t squeeze_proximity;
	uint8_t unknown1[2]; // Always 0x0001
	uint8_t buttons[3];
	uint8_t unknown2; // Always 0x00
	__le32 seq_no;
	__le16 gyro[3];
	__le16 accel[3];
	__le32 imu_ticks;
	uint8_t temperature;
	uint8_t unknown3[7];
	uint8_t trigger_feedback_state;
	uint8_t trigger_feedback_mode;
	uint8_t battery_state; // High bits charge level 0x00-0x0a, low bits battery state
	uint8_t plug_state;    // Flags for USB data and/or power connected
	__le32 host_timestamp;
	__le32 device_timestamp;
	uint8_t unknown4[4];
	uint8_t aes_cmac[8];
	uint8_t unknown5;
	uint8_t crc_failure_count;
	uint8_t padding[7];
	__le32 crc;
};
static_assert(sizeof(struct pssense_input_report) == INPUT_REPORT_LENGTH, "Incorrect input report struct length");

#define PS5_OUTPUT_REPORT_LENGTH 78

enum pssense_output_settings_flag1
{
	PSSENSE_OUTPUT_SETTINGS_FLAG1_UNK0 = 1 << 0,
	PSSENSE_OUTPUT_SETTINGS_FLAG1_RUMBLE_EMULATION = 1 << 1,
	PSSENSE_OUTPUT_SETTINGS_FLAG1_ADAPTIVE_TRIGGER_ENABLE = 1 << 2,
	PSSENSE_OUTPUT_SETTINGS_FLAG1_INTENSITY_INCREASE_SET_ENABLE = 1 << 3,
	PSSENSE_OUTPUT_SETTINGS_FLAG1_INTENSITY_REDUCTION_SET_ENABLE = 1 << 4,
	PSSENSE_OUTPUT_SETTINGS_FLAG1_UNK5 = 1 << 5,
	PSSENSE_OUTPUT_SETTINGS_FLAG1_UNK6 = 1 << 6,
	PSSENSE_OUTPUT_SETTINGS_FLAG1_UNK7 = 1 << 7,
};

#define OUTPUT_SETTINGS_ENABLE_VIBRATION_BITS                                                                          \
	(PSSENSE_OUTPUT_SETTINGS_FLAG1_UNK0 | PSSENSE_OUTPUT_SETTINGS_FLAG1_RUMBLE_EMULATION)

#define OUTPUT_SETTINGS_VIBRATE_MODE_HIGH_120HZ 0x00
// 0x20
#define OUTPUT_SETTINGS_VIBRATE_MODE_LOW_60HZ (PSSENSE_OUTPUT_SETTINGS_FLAG1_UNK5)
// 0x40
#define OUTPUT_SETTINGS_VIBRATE_MODE_CLASSIC_RUMBLE (PSSENSE_OUTPUT_SETTINGS_FLAG1_UNK6)
// 0x60
#define OUTPUT_SETTINGS_VIBRATE_MODE_DIET_RUMBLE                                                                       \
	(PSSENSE_OUTPUT_SETTINGS_FLAG1_UNK5 | PSSENSE_OUTPUT_SETTINGS_FLAG1_UNK6)

enum pssense_output_settings_flag2
{
	PSSENSE_OUTPUT_SETTINGS_FLAG2_UNK0 = 1 << 0,
	PSSENSE_OUTPUT_SETTINGS_FLAG2_UNK1 = 1 << 1,
	//! Used to mark whether to read the status LED enable bool or not.
	PSSENSE_OUTPUT_SETTINGS_FLAG2_STATUS_LED_SET_ENABLE = 1 << 2,
	PSSENSE_OUTPUT_SETTINGS_FLAG2_UNK3 = 1 << 3,
	PSSENSE_OUTPUT_SETTINGS_FLAG2_UNK4 = 1 << 4,
	PSSENSE_OUTPUT_SETTINGS_FLAG2_UNK5 = 1 << 5,
	PSSENSE_OUTPUT_SETTINGS_FLAG2_UNK6 = 1 << 6,
	PSSENSE_OUTPUT_SETTINGS_FLAG2_UNK7 = 1 << 7,
};

enum pssense_adaptive_trigger_mode
{
	TRIGGER_FEEDBACK_MODE_OFF = 0x05,

	// simple versions of other effects, shouldn't be used
	TRIGGER_FEEDBACK_MODE_SIMPLE_FEEDBACK = 0x01,
	TRIGGER_FEEDBACK_MODE_SIMPLE_WEAPON = 0x02,
	TRIGGER_FEEDBACK_MODE_SIMPLE_VIBRATION = 0x06,

	// limited versions of official
	TRIGGER_FEEDBACK_MODE_LIMITED_FEEDBACK = 0x11,
	TRIGGER_FEEDBACK_MODE_LIMITED_WEAPON = 0x12,

	// official ones found through reverse engineering
	TRIGGER_FEEDBACK_MODE_FEEDBACK = 0x21,
	TRIGGER_FEEDBACK_MODE_SLOPE_FEEDBACK = 0x22, // aka Bow
	TRIGGER_FEEDBACK_MODE_WEAPON = 0x25,
	TRIGGER_FEEDBACK_MODE_VIBRATION = 0x26,

	// unofficial ones found through fuzzing firmware
	TRIGGER_FEEDBACK_MODE_GALLOPING = 0x23,
	TRIGGER_FEEDBACK_MODE_MACHINE = 0x27,
};

struct pssense_output_adaptive_trigger_settings
{
	uint8_t mode; // See enum pssense_adaptive_trigger_mode
	union {
		uint8_t raw_parameters[10];
	};
};

struct pssense_led_settings
{
	uint8_t phase;
	uint8_t sequence_number;
	uint8_t period_id;
	__le32 cycle_position;
	__le32 cycle_length;
	uint8_t led_blink[4];
};

struct pssense_output_settings
{
	//! See enum pssense_output_settings_flag1
	uint8_t flag1;
	//! See enum pssense_output_settings_flag2
	uint8_t flag2;
	//! Vibration amplitude from 0x00-0xff. Sending 0 turns vibration off.
	uint8_t vibration_amplitude;
	//! Sony driver sometimes sets to 0x82.
	uint8_t unk0;
	//! Settings for the adaptive trigger
	struct pssense_output_adaptive_trigger_settings trigger_settings;
	//! Time of packet send, in host time, in microseconds
	__le32 host_timestamp_send_time_us;
	//! Settings of the tracking LEDs.
	struct pssense_led_settings led_settings;
	//! Lower 4 bits for haptics reduction, upper 4 bits for trigger reduction. Decreases in 12.5% increments.
	uint8_t trigger_haptics_reduction;
	//! Whether to enable the status LED or not.
	uint8_t status_led_enable;
	uint8_t unk1[2];
};
static_assert(sizeof(struct pssense_output_settings) == 38, "Incorrect output settings struct length");

/**
 * HID output report data packet matching the PS5 layout, with PCM haptics.
 *
 * Reference:
 * https://github.com/BnuuySolutions/PSVR2Toolkit/blob/b6ffe9f03cb2456d9d07aea74e839d9c5fd188f5/projects/psvr2_openvr_driver_ex/sense_controller.h#L44
 */
struct pssense_ps5_output_report
{
	uint8_t report_id;
	uint8_t seq_no_mode; // High bits only; low bits are always 0
	uint8_t tag;         // Needs to be 0x10 for this report
	struct pssense_output_settings settings;
	uint8_t counter;
	uint8_t haptics[32];
	__le32 crc;
};
static_assert(sizeof(struct pssense_ps5_output_report) == PS5_OUTPUT_REPORT_LENGTH,
              "Incorrect output report struct length");

#define FEATURE_REPORT_LENGTH 64
#define CALIBRATION_DATA_LENGTH 116

/**
 * HID output report data packet.
 */
struct pssense_feature_report
{
	uint8_t report_id;
	uint8_t part_id;
	uint8_t data[CALIBRATION_DATA_LENGTH / 2];
	__le32 crc;
};
static_assert(sizeof(struct pssense_feature_report) == FEATURE_REPORT_LENGTH, "Incorrect feature report struct length");

#pragma pack(pop)
