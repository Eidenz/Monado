// Copyright 2023, Collabora, Ltd.
// Copyright 2023, Jarett Millard
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PlayStation Sense controller prober and driver code.
 * @author Jarett Millard <jarett.millard@gmail.com>
 * @ingroup drv_pssense
 */

#include "xrt/xrt_prober.h"

#include "os/os_threading.h"
#include "os/os_hid.h"
#include "os/os_time.h"

#include "math/m_api.h"

#include "tracking/t_imu.h"

#include "util/u_var.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_logging.h"
#include "util/u_trace_marker.h"
#include "util/u_linux.h"
#include "util/u_resampler.h"

#include "math/m_mathinclude.h"
#include "math/m_space.h"
#include "math/m_imu_3dof.h"

#include "pssense_interface.h"
#include "pssense_protocol.h"

#include <stdio.h>
#include <errno.h>


/*!
 * @addtogroup drv_pssense
 * @{
 */

#define PSSENSE_TRACE(p, ...) U_LOG_XDEV_IFL_T(&p->base, p->log_level, __VA_ARGS__)
#define PSSENSE_DEBUG(p, ...) U_LOG_XDEV_IFL_D(&p->base, p->log_level, __VA_ARGS__)
#define PSSENSE_WARN(p, ...) U_LOG_XDEV_IFL_W(&p->base, p->log_level, __VA_ARGS__)
#define PSSENSE_ERROR(p, ...) U_LOG_XDEV_IFL_E(&p->base, p->log_level, __VA_ARGS__)

DEBUG_GET_ONCE_LOG_OPTION(pssense_log, "PSSENSE_LOG", U_LOGGING_INFO)

static struct xrt_binding_input_pair touch_inputs_pssense[] = {
    {XRT_INPUT_TOUCH_X_CLICK, XRT_INPUT_PSSENSE_SQUARE_CLICK},
    {XRT_INPUT_TOUCH_X_TOUCH, XRT_INPUT_PSSENSE_SQUARE_TOUCH},
    {XRT_INPUT_TOUCH_Y_CLICK, XRT_INPUT_PSSENSE_TRIANGLE_CLICK},
    {XRT_INPUT_TOUCH_Y_TOUCH, XRT_INPUT_PSSENSE_TRIANGLE_TOUCH},
    {XRT_INPUT_TOUCH_MENU_CLICK, XRT_INPUT_PSSENSE_PS_CLICK},
    {XRT_INPUT_TOUCH_A_CLICK, XRT_INPUT_PSSENSE_CROSS_CLICK},
    {XRT_INPUT_TOUCH_A_TOUCH, XRT_INPUT_PSSENSE_CROSS_TOUCH},
    {XRT_INPUT_TOUCH_B_CLICK, XRT_INPUT_PSSENSE_CIRCLE_CLICK},
    {XRT_INPUT_TOUCH_B_TOUCH, XRT_INPUT_PSSENSE_CIRCLE_TOUCH},
    {XRT_INPUT_TOUCH_SYSTEM_CLICK, XRT_INPUT_PSSENSE_PS_CLICK},
    {XRT_INPUT_TOUCH_SQUEEZE_VALUE, XRT_INPUT_PSSENSE_SQUEEZE_CLICK},
    {XRT_INPUT_TOUCH_TRIGGER_TOUCH, XRT_INPUT_PSSENSE_TRIGGER_TOUCH},
    {XRT_INPUT_TOUCH_TRIGGER_VALUE, XRT_INPUT_PSSENSE_TRIGGER_VALUE},
    {XRT_INPUT_TOUCH_THUMBSTICK_CLICK, XRT_INPUT_PSSENSE_THUMBSTICK_CLICK},
    {XRT_INPUT_TOUCH_THUMBSTICK_TOUCH, XRT_INPUT_PSSENSE_THUMBSTICK_TOUCH},
    {XRT_INPUT_TOUCH_THUMBSTICK, XRT_INPUT_PSSENSE_THUMBSTICK},
    {XRT_INPUT_TOUCH_GRIP_POSE, XRT_INPUT_PSSENSE_GRIP_POSE},
    {XRT_INPUT_TOUCH_AIM_POSE, XRT_INPUT_PSSENSE_AIM_POSE},
    {XRT_INPUT_TOUCH_TRIGGER_PROXIMITY, XRT_INPUT_PSSENSE_TRIGGER_PROXIMITY},
};

static struct xrt_binding_output_pair touch_outputs_pssense[] = {
    {XRT_OUTPUT_NAME_TOUCH_HAPTIC, XRT_OUTPUT_NAME_PSSENSE_VIBRATION},
};

static struct xrt_binding_input_pair simple_inputs_pssense[] = {
    {XRT_INPUT_SIMPLE_SELECT_CLICK, XRT_INPUT_PSSENSE_TRIGGER_VALUE},
    {XRT_INPUT_SIMPLE_MENU_CLICK, XRT_INPUT_PSSENSE_OPTIONS_CLICK},
    {XRT_INPUT_SIMPLE_GRIP_POSE, XRT_INPUT_PSSENSE_GRIP_POSE},
    {XRT_INPUT_SIMPLE_AIM_POSE, XRT_INPUT_PSSENSE_AIM_POSE},
};

static struct xrt_binding_output_pair simple_outputs_pssense[] = {
    {XRT_OUTPUT_NAME_SIMPLE_VIBRATION, XRT_OUTPUT_NAME_PSSENSE_VIBRATION},
};

static struct xrt_binding_profile binding_profiles_pssense[] = {
    {
        .name = XRT_DEVICE_TOUCH_CONTROLLER,
        .inputs = touch_inputs_pssense,
        .input_count = ARRAY_SIZE(touch_inputs_pssense),
        .outputs = touch_outputs_pssense,
        .output_count = ARRAY_SIZE(touch_outputs_pssense),
    },
    {
        .name = XRT_DEVICE_SIMPLE_CONTROLLER,
        .inputs = simple_inputs_pssense,
        .input_count = ARRAY_SIZE(simple_inputs_pssense),
        .outputs = simple_outputs_pssense,
        .output_count = ARRAY_SIZE(simple_outputs_pssense),
    },
};

/*!
 * Indices where each input is in the input list.
 */
enum pssense_input_index
{
	PSSENSE_INDEX_PS_CLICK,
	PSSENSE_INDEX_SHARE_CLICK,
	PSSENSE_INDEX_OPTIONS_CLICK,
	PSSENSE_INDEX_SQUARE_CLICK,
	PSSENSE_INDEX_SQUARE_TOUCH,
	PSSENSE_INDEX_TRIANGLE_CLICK,
	PSSENSE_INDEX_TRIANGLE_TOUCH,
	PSSENSE_INDEX_CROSS_CLICK,
	PSSENSE_INDEX_CROSS_TOUCH,
	PSSENSE_INDEX_CIRCLE_CLICK,
	PSSENSE_INDEX_CIRCLE_TOUCH,
	PSSENSE_INDEX_SQUEEZE_CLICK,
	PSSENSE_INDEX_SQUEEZE_TOUCH,
	PSSENSE_INDEX_SQUEEZE_PROXIMITY,
	PSSENSE_INDEX_SQUEEZE_PROXIMITY_FLOAT,
	PSSENSE_INDEX_TRIGGER_CLICK,
	PSSENSE_INDEX_TRIGGER_TOUCH,
	PSSENSE_INDEX_TRIGGER_VALUE,
	PSSENSE_INDEX_TRIGGER_PROXIMITY,
	PSSENSE_INDEX_TRIGGER_PROXIMITY_FLOAT,
	PSSENSE_INDEX_THUMBSTICK,
	PSSENSE_INDEX_THUMBSTICK_CLICK,
	PSSENSE_INDEX_THUMBSTICK_TOUCH,
	PSSENSE_INDEX_GRIP_POSE,
	PSSENSE_INDEX_AIM_POSE,
	PSSENSE_INPUT_COUNT,
};

/*!
 * PlayStation Sense state parsed from a data packet.
 */
struct pssense_input_state
{
	uint64_t timestamp_ns;
	uint32_t seq_no;

	bool ps_click;
	bool share_click;
	bool options_click;
	bool square_click;
	bool square_touch;
	bool triangle_click;
	bool triangle_touch;
	bool cross_click;
	bool cross_touch;
	bool circle_click;
	bool circle_touch;
	bool squeeze_click;
	bool squeeze_touch;
	float squeeze_proximity;
	bool trigger_click;
	bool trigger_touch;
	float trigger_value;
	float trigger_proximity;
	bool thumbstick_click;
	bool thumbstick_touch;
	struct xrt_vec2 thumbstick;

	uint32_t imu_ticks_last;
	uint64_t imu_ticks_total;

	struct xrt_vec3_i32 gyro_raw;
	struct xrt_vec3_i32 accel_raw;

	bool battery_state_valid;
	bool battery_charging;
	//! Charge level from 0..1
	float battery_charge_percent;
};

/*!
 * A single PlayStation Sense Controller.
 *
 * @implements xrt_device
 */
struct pssense_device
{
	struct xrt_device base;

	struct os_hid_device *hid;
	struct os_thread_helper controller_thread;

	enum
	{
		PSSENSE_HAND_LEFT,
		PSSENSE_HAND_RIGHT
	} hand;

	enum u_logging_level log_level;

	struct os_precise_sleeper sleeper;

	//! Input state parsed from most recent packet
	struct pssense_input_state state;
	//! Pending output state to send to device
	struct
	{
		uint8_t next_seq_no;
		uint8_t packet_counter;

		struct u_resampler *pcm_haptics_resampler;

		bool send_vibration;
		uint8_t vibration_amplitude;
		uint8_t vibration_mode;

		uint64_t vibration_end_timestamp_ns;

		bool send_trigger_feedback;
		enum pssense_adaptive_trigger_mode trigger_feedback_mode;
	} output;

	struct m_imu_3dof fusion;
	struct xrt_pose pose;

	struct
	{
		bool button_states;
		bool tracking;
	} gui;
};

const uint32_t CRC_POLYNOMIAL = 0xedb88320;

/*
 *
 * Internal functions
 *
 */

static uint32_t
crc32_le(uint32_t crc, uint8_t const *p, size_t len)
{
	int i;
	crc ^= 0xffffffff;
	while (len--) {
		crc ^= *p++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? CRC_POLYNOMIAL : 0);
	}
	return crc ^ 0xffffffff;
}

/*!
 * Reads one packet from the device, handles time out, locking and checking if
 * the thread has been told to shut down.
 */
static int
pssense_read_packet_data(struct pssense_device *pssense, uint8_t *buffer, size_t size, bool check_size)
{
	// Poll, don't block. Outer thread needs to run quick
	int ret = os_hid_read(pssense->hid, buffer, size, 0);

	// No data yet
	if (ret == 0) {
		return -EAGAIN;
	}

	if (ret < 0) {
		PSSENSE_ERROR(pssense, "Failed to read device '%i'!", ret);
		return ret;
	}

	// Skip this check if we haven't flushed all the compat mode packets yet, since they're shorter.
	if (check_size && ret != (int)size) {
		PSSENSE_ERROR(pssense, "Unexpected HID packet size %i (expected %zu)", ret, size);
		return -EIO;
	}

	return ret;
}

static void
pssense_update_fusion(struct pssense_device *pssense)
{
	struct xrt_vec3 gyro;
	gyro.x = DEG_TO_RAD(pssense->state.gyro_raw.x * PSSENSE_GYRO_SCALE_DEG);
	gyro.y = DEG_TO_RAD(pssense->state.gyro_raw.y * PSSENSE_GYRO_SCALE_DEG);
	gyro.z = DEG_TO_RAD(pssense->state.gyro_raw.z * PSSENSE_GYRO_SCALE_DEG);

	struct xrt_vec3 accel;
	accel.x = pssense->state.accel_raw.x * PSSENSE_ACCEL_SCALE;
	accel.y = pssense->state.accel_raw.y * PSSENSE_ACCEL_SCALE;
	accel.z = pssense->state.accel_raw.z * PSSENSE_ACCEL_SCALE;

	// TODO: Apply correction from calibration data

	// Each IMU tick is .33μs
	m_imu_3dof_update(&pssense->fusion, pssense->state.imu_ticks_total * NS_PER_IMU_TICK, &accel, &gyro);
	pssense->pose.orientation = pssense->fusion.rot;
}

static bool
pssense_handle_read(struct pssense_device *pssense)
{
	int ret;

	// Report data
	struct pssense_input_report data = {0};
	ret = pssense_read_packet_data(pssense, (uint8_t *)&data, sizeof(data), true);

	if (ret == -EAGAIN) {
		// No data yet, not an error
		return true;
	}

	if (ret < 0) {
		PSSENSE_ERROR(pssense, "Error reading from device: %d", ret);
		return false;
	}

	// Final input state
	struct pssense_input_state input = {
	    .timestamp_ns = os_monotonic_get_ns(),
	};

	if (data.report_id != INPUT_REPORT_ID) {
		PSSENSE_WARN(pssense, "Unrecognized HID report id %u", data.report_id);
		return false;
	}

	// Verify the CRC of the packet
	uint32_t expected_crc = __le32_to_cpu(data.crc);
	uint32_t crc = crc32_le(0, &INPUT_REPORT_CRC32_SEED, 1);
	crc = crc32_le(crc, (uint8_t *)&data, sizeof(struct pssense_input_report) - 4);
	if (crc != expected_crc) {
		PSSENSE_WARN(pssense, "CRC mismatch; skipping input. Expected %08X but got %08X", expected_crc, crc);
		return false;
	}

	uint32_t seq_no = __le32_to_cpu(data.seq_no);
	if (input.seq_no != 0 && seq_no != input.seq_no + 1) {
		PSSENSE_WARN(pssense, "Missed seq no %u. Previous was %u", seq_no, input.seq_no);
	}
	input.seq_no = seq_no;

	// Update input state
	input.ps_click = (data.buttons[1] & 16) != 0;
	input.squeeze_touch = (data.buttons[2] & 8) != 0;
	input.squeeze_proximity = data.squeeze_proximity / 255.0f;
	input.trigger_touch = (data.buttons[1] & 128) != 0;
	input.trigger_value = data.trigger_value / 255.0f;
	input.trigger_proximity = data.trigger_proximity / 255.0f;
	input.thumbstick.x = (data.thumbstick_x - 128) / 128.0f;
	input.thumbstick.y = (data.thumbstick_y - 128) / -128.0f;
	input.thumbstick_touch = (data.buttons[2] & 4) != 0;

	if (pssense->hand == PSSENSE_HAND_LEFT) {
		input.share_click = (data.buttons[1] & 1) != 0;
		input.square_click = (data.buttons[0] & 1) != 0;
		input.square_touch = (data.buttons[2] & 2) != 0;
		input.triangle_click = (data.buttons[0] & 8) != 0;
		input.triangle_touch = (data.buttons[2] & 1) != 0;
		input.squeeze_click = (data.buttons[0] & 16) != 0;
		input.trigger_click = (data.buttons[0] & 64) != 0;
		input.thumbstick_click = (data.buttons[1] & 4) != 0;
	} else if (pssense->hand == PSSENSE_HAND_RIGHT) {
		input.options_click = (data.buttons[1] & 2) != 0;
		input.cross_click = (data.buttons[0] & 2) != 0;
		input.cross_touch = (data.buttons[2] & 2) != 0;
		input.circle_click = (data.buttons[0] & 4) != 0;
		input.circle_touch = (data.buttons[2] & 1) != 0;
		input.squeeze_click = (data.buttons[0] & 32) != 0;
		input.trigger_click = (data.buttons[0] & 128) != 0;
		input.thumbstick_click = (data.buttons[1] & 8) != 0;
	}

	// Update IMU data
	uint32_t imu_ticks = __le32_to_cpu(data.imu_ticks);
	int64_t imu_ticks_delta = imu_ticks - input.imu_ticks_last;
	if (imu_ticks_delta >= 0) {
		input.imu_ticks_total += imu_ticks_delta;
		input.imu_ticks_last = imu_ticks;

		input.gyro_raw.x = (int16_t)__le16_to_cpu(data.gyro[0]);
		input.gyro_raw.y = (int16_t)__le16_to_cpu(data.gyro[1]);
		input.gyro_raw.z = (int16_t)__le16_to_cpu(data.gyro[2]);

		input.accel_raw.x = (int16_t)__le16_to_cpu(data.accel[0]);
		input.accel_raw.y = (int16_t)__le16_to_cpu(data.accel[1]);
		input.accel_raw.z = (int16_t)__le16_to_cpu(data.accel[2]);
	} else {
		PSSENSE_WARN(pssense, "Time went backwards. Check your play area for black holes.");
	}

	// Battery state is upper 4 bits
	uint8_t battery_state = data.battery_state >> 4;

	// Charge values go from 0..10, so add 5% and cap at 100% so we never show 0% charge
	float battery_percent = MIN(1.0f, (data.battery_state & 0xf) * .1f + .05);

	bool battery_state_valid, charging;
	if (battery_state == CHARGE_STATE_DISCHARGING) {
		battery_state_valid = true;
		charging = false;
	} else if (battery_state == CHARGE_STATE_CHARGING) {
		battery_state_valid = true;
		charging = true;
	} else if (battery_state == CHARGE_STATE_FULL) {
		battery_state_valid = true;
		charging = true;
		battery_percent = 1.0f;
	} else if (battery_state == CHARGE_STATE_ABNORMAL_VOLTAGE) {
		battery_state_valid = false;
		PSSENSE_WARN(pssense, "Unable to determine charge state: abnormal voltage");
	} else if (battery_state == CHARGE_STATE_ABNORMAL_TEMP) {
		battery_state_valid = false;
		PSSENSE_WARN(pssense, "Unable to determine charge state: abnormal temp");
	} else if (battery_state == CHARGE_STATE_CHARGING_ERROR) {
		battery_state_valid = false;
		PSSENSE_WARN(pssense, "Unable to determine charge state: charging error");
	} else {
		battery_state_valid = false;
		PSSENSE_WARN(pssense, "Unable to determine charge state: unknown reason");
	}

	input.battery_state_valid = battery_state_valid;
	if (battery_state_valid) {
		if (charging != input.battery_charging || battery_percent != input.battery_charge_percent) {
			PSSENSE_TRACE(pssense, "Battery at %.f%%, %s", battery_percent * 100,
			              charging ? "charging" : "discharging");
		}
		input.battery_charging = charging;
		input.battery_charge_percent = battery_percent;
	}

	os_thread_helper_lock(&pssense->controller_thread);

	pssense->state = input;
	pssense_update_fusion(pssense);

	os_thread_helper_unlock(&pssense->controller_thread);

	return true;
}

static int
pssense_send_output_report_locked(struct pssense_device *pssense)
{
	uint64_t timestamp_ns = os_monotonic_get_ns();

	struct pssense_ps5_output_report report = {
	    .report_id = OUTPUT_REPORT_ID,
	    // low bits are always zero, to indicate we are using the PS5 packet format
	    .seq_no_mode = (pssense->output.next_seq_no << 4) | (0x0),
	    .tag = OUTPUT_REPORT_TAG,
	    // Packet counter needs to increment with every packet, or PCM haptics won't work.
	    .counter = pssense->output.packet_counter++,
	};

	float pcm_buf[PCM_HAPTIC_BUF_SIZE] = {0};
	size_t read_pcm_samples = u_resampler_read(pssense->output.pcm_haptics_resampler, pcm_buf, ARRAY_SIZE(pcm_buf));

	if (timestamp_ns >= pssense->output.vibration_end_timestamp_ns) {
		pssense->output.vibration_amplitude = 0;
	}

	if (read_pcm_samples > 0) {
		for (size_t i = 0; i < read_pcm_samples; i++) {
			// Convert from float [-1, 1] to int8 [-128, 127].
			report.haptics[i] = (int8_t)(CLAMP(((pcm_buf[i] + 1.0f) * 0.5f * 255) - 128, -128, 127));
		}
	} else if (pssense->output.send_vibration) {
		report.settings.flag1 |= OUTPUT_SETTINGS_ENABLE_VIBRATION_BITS | pssense->output.vibration_mode;
		report.settings.vibration_amplitude = pssense->output.vibration_amplitude;
		pssense->output.send_vibration = pssense->output.vibration_amplitude > 0;
	}

	if (pssense->output.send_trigger_feedback) {
		report.settings.flag1 |= PSSENSE_OUTPUT_SETTINGS_FLAG1_ADAPTIVE_TRIGGER_ENABLE;
		report.settings.trigger_settings.mode = pssense->output.trigger_feedback_mode;
		pssense->output.send_trigger_feedback = false;
	}

	report.settings.host_timestamp_send_time_us = __cpu_to_le32(timestamp_ns / U_TIME_1US_IN_NS);

	pssense->output.next_seq_no = (pssense->output.next_seq_no + 1) % 16;

	uint32_t crc = crc32_le(0, &OUTPUT_REPORT_CRC32_SEED, 1);
	crc = crc32_le(crc, (uint8_t *)&report, sizeof(struct pssense_ps5_output_report) - 4);
	report.crc = __cpu_to_le32(crc);

	PSSENSE_TRACE(pssense,
	              "Setting vibration amplitude: %u, mode: %02X, trigger feedback mode: %02X. Next seq no: %u. PCM "
	              "samples: %zu",
	              pssense->output.vibration_amplitude, pssense->output.vibration_mode,
	              pssense->output.trigger_feedback_mode, pssense->output.next_seq_no, read_pcm_samples);
	int ret = os_hid_write(pssense->hid, (uint8_t *)&report, sizeof(struct pssense_ps5_output_report));
	if (ret != sizeof(struct pssense_ps5_output_report)) {
		PSSENSE_WARN(pssense, "Failed to send output report: %d", ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static void *
pssense_run_thread(void *ptr)
{
	U_TRACE_SET_THREAD_NAME("PS Sense");

	struct pssense_device *pssense = (struct pssense_device *)ptr;

#ifdef XRT_OS_LINUX
	u_linux_try_to_set_realtime_priority_on_thread(pssense->log_level, "PS Sense");
#endif

	union {
		uint8_t buffer[sizeof(struct pssense_input_report)];
		struct pssense_input_report report;
	} data;

	// The Sense controller starts in compat mode with a different HID report ID and format.
	// We need to discard packets until we get a correct report.
	while (pssense_read_packet_data(pssense, data.buffer, sizeof(data), false) &&
	       data.report.report_id != INPUT_REPORT_ID) {
		PSSENSE_TRACE(pssense, "Discarding compat mode HID report");
	}

	os_thread_helper_lock(&pssense->controller_thread);

	// 32/3000hz (PCM haptic rate), this will *technically* run slightly fast, but like, that's fine.
	const time_duration_ns pcm_haptics_period_ns = 10666666;

	timepoint_ns next_output_ns = os_monotonic_get_ns();

	int result;
	while (os_thread_helper_is_running_locked(&pssense->controller_thread) && result >= 0) {
		os_thread_helper_unlock(&pssense->controller_thread);

		result = pssense_handle_read(pssense);

		if (result >= 0) {
			timepoint_ns now = os_monotonic_get_ns();

			if (now >= next_output_ns) {
				os_thread_helper_lock(&pssense->controller_thread);
				result = pssense_send_output_report_locked(pssense);
				os_thread_helper_unlock(&pssense->controller_thread);

				next_output_ns = next_output_ns + pcm_haptics_period_ns;
			}
		}

		if (result >= 0) {
			// @note we don't break the earlier `now` out into the outer scope accessible from here since
			//       sending may take some non-negligible amount of time.
			timepoint_ns to_next_output = next_output_ns - os_monotonic_get_ns();

			// Only sleep if it's an increment greater than 50us, Linux doesn't like sleeping that short and
			// often oversleeps a bit, timing matters with PCM haptics and LED sync!
			if (to_next_output > (U_TIME_1US_IN_NS * 50L)) {
				// Sleep 1ms, or half the time to the next output report, whichever is smaller. We want
				// to wake up frequently enough to read incoming packets, but not so much that we waste
				// CPU time waking too often.
				os_precise_sleeper_nanosleep(&pssense->sleeper,
				                             MIN(to_next_output / 2, U_TIME_1MS_IN_NS));
			}
		}

		os_thread_helper_lock(&pssense->controller_thread);
	}

	os_thread_helper_unlock(&pssense->controller_thread);

	return NULL;
}

static void
pssense_get_fusion_pose(struct pssense_device *pssense,
                        enum xrt_input_name name,
                        int64_t at_timestamp_ns,
                        struct xrt_space_relation *out_relation)
{
	out_relation->pose = pssense->pose;
	out_relation->linear_velocity.x = 0.0f;
	out_relation->linear_velocity.y = 0.0f;
	out_relation->linear_velocity.z = 0.0f;

	/*!
	 * @todo This is hack, fusion reports angvel relative to the device but
	 * it needs to be in relation to the base space. Rotating it with the
	 * device orientation is enough to get it into the right space, angular
	 * velocity is a derivative so needs a special rotation.
	 */
	math_quat_rotate_derivative(&pssense->pose.orientation, &pssense->fusion.last.gyro,
	                            &out_relation->angular_velocity);

	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	    XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);
}

/*!
 * Retrieving the calibration data report will switch the Sense controller from compat mode into full mode.
 */
bool
pssense_get_calibration_data(struct pssense_device *pssense)
{
	uint8_t calibration_data[CALIBRATION_DATA_LENGTH] = {0};

	bool invalid_crc;
	do {
		invalid_crc = false;

		// Calibration has to be read in two parts with two feature reads.
		for (int i = 0; i < 2; i++) {
			// no need for initialization, we assert whole size is read
			struct pssense_feature_report report_buffer;
			int ret = os_hid_get_feature(pssense->hid, CALIBRATION_DATA_FEATURE_REPORT_ID,
			                             (uint8_t *)&report_buffer, sizeof(report_buffer));

			if (ret < 0) {
				PSSENSE_ERROR(pssense, "Failed to retrieve calibration report: %d", ret);
				return false;
			}

			if (ret != sizeof(report_buffer)) {
				PSSENSE_ERROR(pssense, "Invalid byte count transferred, expected %zu got %d",
				              sizeof(report_buffer), ret);
				return false;
			}

			switch (report_buffer.part_id) {
			case CALIBRATION_DATA_PART_ID_1: {
				memcpy(calibration_data, report_buffer.data, sizeof(report_buffer.data));
				break;
			}
			case CALIBRATION_DATA_PART_ID_2: {
				memcpy(calibration_data + sizeof(report_buffer.data), report_buffer.data,
				       sizeof(report_buffer.data));
				break;
			}
			default: {
				PSSENSE_ERROR(pssense, "Unknown calibration data part ID %u", report_buffer.part_id);
				return false;
			}
			}

			uint32_t crc = crc32_le(0, &FEATURE_REPORT_CRC32_SEED, 1);
			crc = crc32_le(crc, (uint8_t *)&report_buffer, sizeof(report_buffer) - 4);
			uint32_t expected_crc = __le32_to_cpu(report_buffer.crc);

			if (crc != expected_crc) {
				PSSENSE_WARN(pssense, "Invalid feature report CRC. Expected 0x%08X, actual 0x%08X",
				             expected_crc, crc);
				invalid_crc = true;
			}
		}
	} while (invalid_crc);

	// TODO: Parse calibration data into prefiler

	return true;
}

static uint64_t
saturating_add_uint64(uint64_t a, uint64_t b)
{
	if (UINT64_MAX - a < b) {
		return UINT64_MAX;
	} else {
		return a + b;
	}
}

/*
 *
 * Driver implementations
 *
 */

static void
pssense_device_destroy(struct xrt_device *xdev)
{
	struct pssense_device *pssense = (struct pssense_device *)xdev;

	os_precise_sleeper_deinit(&pssense->sleeper);

	if (pssense->output.pcm_haptics_resampler) {
		u_resampler_destroy(pssense->output.pcm_haptics_resampler);
		pssense->output.pcm_haptics_resampler = NULL;
	}

	// Destroy the thread object.
	os_thread_helper_destroy(&pssense->controller_thread);

	m_imu_3dof_close(&pssense->fusion);

	// Remove the variable tracking.
	u_var_remove_root(pssense);

	if (pssense->hid != NULL) {
		os_hid_destroy(pssense->hid);
		pssense->hid = NULL;
	}

	free(pssense);
}

static xrt_result_t
pssense_device_update_inputs(struct xrt_device *xdev)
{
	struct pssense_device *pssense = (struct pssense_device *)xdev;

	// Lock the data.
	os_thread_helper_lock(&pssense->controller_thread);

	for (uint32_t i = 0; i < (uint32_t)sizeof(enum pssense_input_index); i++) {
		pssense->base.inputs[i].timestamp = (int64_t)pssense->state.timestamp_ns;
	}
	pssense->base.inputs[PSSENSE_INDEX_PS_CLICK].value.boolean = pssense->state.ps_click;
	pssense->base.inputs[PSSENSE_INDEX_SHARE_CLICK].value.boolean = pssense->state.share_click;
	pssense->base.inputs[PSSENSE_INDEX_OPTIONS_CLICK].value.boolean = pssense->state.options_click;
	pssense->base.inputs[PSSENSE_INDEX_SQUARE_CLICK].value.boolean = pssense->state.square_click;
	pssense->base.inputs[PSSENSE_INDEX_SQUARE_TOUCH].value.boolean = pssense->state.square_touch;
	pssense->base.inputs[PSSENSE_INDEX_TRIANGLE_CLICK].value.boolean = pssense->state.triangle_click;
	pssense->base.inputs[PSSENSE_INDEX_TRIANGLE_TOUCH].value.boolean = pssense->state.triangle_touch;
	pssense->base.inputs[PSSENSE_INDEX_CROSS_CLICK].value.boolean = pssense->state.cross_click;
	pssense->base.inputs[PSSENSE_INDEX_CROSS_TOUCH].value.boolean = pssense->state.cross_touch;
	pssense->base.inputs[PSSENSE_INDEX_CIRCLE_CLICK].value.boolean = pssense->state.circle_click;
	pssense->base.inputs[PSSENSE_INDEX_CIRCLE_TOUCH].value.boolean = pssense->state.circle_touch;
	pssense->base.inputs[PSSENSE_INDEX_SQUEEZE_CLICK].value.boolean = pssense->state.squeeze_click;
	pssense->base.inputs[PSSENSE_INDEX_SQUEEZE_TOUCH].value.boolean = pssense->state.squeeze_touch;
	pssense->base.inputs[PSSENSE_INDEX_SQUEEZE_PROXIMITY].value.boolean = pssense->state.squeeze_proximity > 0.7f;
	pssense->base.inputs[PSSENSE_INDEX_SQUEEZE_PROXIMITY_FLOAT].value.vec1.x = pssense->state.squeeze_proximity;
	pssense->base.inputs[PSSENSE_INDEX_TRIGGER_CLICK].value.boolean = pssense->state.trigger_click;
	pssense->base.inputs[PSSENSE_INDEX_TRIGGER_TOUCH].value.boolean = pssense->state.trigger_touch;
	pssense->base.inputs[PSSENSE_INDEX_TRIGGER_VALUE].value.vec1.x = pssense->state.trigger_value;
	pssense->base.inputs[PSSENSE_INDEX_TRIGGER_PROXIMITY].value.boolean = pssense->state.trigger_proximity > 0.7f;
	pssense->base.inputs[PSSENSE_INDEX_TRIGGER_PROXIMITY_FLOAT].value.vec1.x = pssense->state.trigger_proximity;
	pssense->base.inputs[PSSENSE_INDEX_THUMBSTICK].value.vec2 = pssense->state.thumbstick;
	pssense->base.inputs[PSSENSE_INDEX_THUMBSTICK_CLICK].value.boolean = pssense->state.thumbstick_click;
	pssense->base.inputs[PSSENSE_INDEX_THUMBSTICK_TOUCH].value.boolean = pssense->state.thumbstick_touch;

	// Done now.
	os_thread_helper_unlock(&pssense->controller_thread);

	return XRT_SUCCESS;
}

static xrt_result_t
set_vibration_output(struct pssense_device *pssense,
                     const struct xrt_output_value *value,
                     bool *send_vibration,
                     uint8_t *vibration_amplitude,
                     uint8_t *vibration_mode)
{
	switch (value->type) {
	case XRT_OUTPUT_VALUE_TYPE_VIBRATION: {
		*send_vibration = true;
		*vibration_amplitude = (uint8_t)(value->vibration.amplitude * 255.0f);
		*vibration_mode = OUTPUT_SETTINGS_VIBRATE_MODE_CLASSIC_RUMBLE;

		if (value->vibration.frequency != XRT_FREQUENCY_UNSPECIFIED) {
			if (value->vibration.frequency <= 70) {
				*vibration_mode = OUTPUT_SETTINGS_VIBRATE_MODE_LOW_60HZ;
			} else if (value->vibration.frequency >= 110) {
				*vibration_mode = OUTPUT_SETTINGS_VIBRATE_MODE_HIGH_120HZ;
			}
		}
		break;
	}
	case XRT_OUTPUT_VALUE_TYPE_PCM_VIBRATION: {
		os_thread_helper_lock(&pssense->controller_thread);
		// Reset the resampler if we're not appending.
		if (!value->pcm_vibration.append) {
			u_resampler_reset(pssense->output.pcm_haptics_resampler);
		}

		size_t samples_consumed =
		    u_resampler_write(pssense->output.pcm_haptics_resampler, value->pcm_vibration.buffer,
		                      value->pcm_vibration.buffer_size, value->pcm_vibration.sample_rate);
		os_thread_helper_unlock(&pssense->controller_thread);

		*value->pcm_vibration.samples_consumed = samples_consumed;
		break;
	}
	default: {
		U_LOG_XDEV_UNSUPPORTED_OUTPUT(&pssense->base, pssense->log_level, XRT_OUTPUT_NAME_PSSENSE_VIBRATION);
		return XRT_ERROR_OUTPUT_UNSUPPORTED;
		break;
	}
	}

	return XRT_SUCCESS;
}

static xrt_result_t
pssense_set_output(struct xrt_device *xdev, enum xrt_output_name name, const struct xrt_output_value *value)
{
	struct pssense_device *pssense = (struct pssense_device *)xdev;

	bool send_vibration = false;
	uint8_t vibration_amplitude;
	uint8_t vibration_mode;

	bool send_trigger_feedback = false;
	enum pssense_adaptive_trigger_mode trigger_feedback_mode;

	switch (name) {
	case XRT_OUTPUT_NAME_PSSENSE_VIBRATION: {
		xrt_result_t result =
		    set_vibration_output(pssense, value, &send_vibration, &vibration_amplitude, &vibration_mode);
		if (result != XRT_SUCCESS) {
			return result;
		}

		break;
	}
	case XRT_OUTPUT_NAME_PSSENSE_TRIGGER_FEEDBACK: {
		for (uint64_t i = 0; i < value->force_feedback.force_feedback_location_count; i++) {
			if (value->force_feedback.force_feedback[i].location ==
			    XRT_FORCE_FEEDBACK_LOCATION_LEFT_INDEX) {
				send_trigger_feedback = true;
				if (value->force_feedback.force_feedback[i].value > 0) {
					trigger_feedback_mode = TRIGGER_FEEDBACK_MODE_SIMPLE_FEEDBACK;
				} else {
					trigger_feedback_mode = TRIGGER_FEEDBACK_MODE_OFF;
				}
			}
		}

		break;
	}
	default: {
		U_LOG_XDEV_UNSUPPORTED_OUTPUT(&pssense->base, pssense->log_level, name);
		return XRT_ERROR_OUTPUT_UNSUPPORTED;
	}
	}

	timepoint_ns now = os_monotonic_get_ns();

	os_thread_helper_lock(&pssense->controller_thread);
	if (send_vibration && (vibration_amplitude != pssense->output.vibration_amplitude ||
	                       vibration_mode != pssense->output.vibration_mode)) {
		pssense->output.send_vibration = true;
		pssense->output.vibration_amplitude = vibration_amplitude;
		pssense->output.vibration_mode = vibration_mode;
		// Some applications (hello_xr has been seen doing this) will set the duration to INT64_MAX, so when
		// adding directly, it overflows and doesn't work. This prevents that.
		pssense->output.vibration_end_timestamp_ns = saturating_add_uint64(now, value->vibration.duration_ns);
	}

	if (send_trigger_feedback && trigger_feedback_mode != pssense->output.trigger_feedback_mode) {
		pssense->output.send_trigger_feedback = true;
		pssense->output.trigger_feedback_mode = trigger_feedback_mode;
	}
	os_thread_helper_unlock(&pssense->controller_thread);

	return XRT_SUCCESS;
}

xrt_result_t
pssense_get_output_limits(struct xrt_device *xdev, struct xrt_output_limits *limits)
{
	(*limits) = XRT_C11_COMPOUND(struct xrt_output_limits){
	    // PCM data is played back at 3000hz
	    .haptic_pcm_sample_rate = PCM_SAMPLE_RATE,
	};

	return XRT_SUCCESS;
}

static xrt_result_t
pssense_get_tracked_pose(struct xrt_device *xdev,
                         enum xrt_input_name name,
                         int64_t at_timestamp_ns,
                         struct xrt_space_relation *out_relation)
{
	struct pssense_device *pssense = (struct pssense_device *)xdev;

	if (name != XRT_INPUT_PSSENSE_AIM_POSE && name != XRT_INPUT_PSSENSE_GRIP_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&pssense->base, pssense->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct xrt_relation_chain xrc = {0};
	struct xrt_pose pose_correction = {0};

	// Rotate the grip/aim pose up by 60 degrees around the X axis
	struct xrt_vec3 axis = {1.0, 0, 0};
	math_quat_from_angle_vector(DEG_TO_RAD(60), &axis, &pose_correction.orientation);
	m_relation_chain_push_pose(&xrc, &pose_correction);

	struct xrt_space_relation *rel = m_relation_chain_reserve(&xrc);

	os_thread_helper_lock(&pssense->controller_thread);
	pssense_get_fusion_pose(pssense, name, at_timestamp_ns, rel);
	os_thread_helper_unlock(&pssense->controller_thread);

	m_relation_chain_resolve(&xrc, out_relation);

	return XRT_SUCCESS;
}

static xrt_result_t
pssense_get_battery_status(struct xrt_device *xdev, bool *out_present, bool *out_charging, float *out_charge)
{
	struct pssense_device *pssense = (struct pssense_device *)xdev;
	if (!pssense->state.battery_state_valid) {
		*out_present = false;
		return XRT_SUCCESS;
	}

	*out_present = true;
	*out_charging = pssense->state.battery_charging;
	*out_charge = pssense->state.battery_charge_percent;

	return XRT_SUCCESS;
}

/*
 *
 * Exported functions
 *
 */

#define SET_INPUT(NAME) (pssense->base.inputs[PSSENSE_INDEX_##NAME].name = XRT_INPUT_PSSENSE_##NAME)

struct xrt_device *
pssense_create(struct xrt_prober *xp, struct xrt_prober_device *xpdev)
{
	struct os_hid_device *hid = NULL;
	int ret;

	ret = xrt_prober_open_hid_interface(xp, xpdev, 0, &hid);
	if (ret != 0) {
		U_LOG_E("Failed to open HID interface for PlayStation Sense controller!");
		return NULL;
	}

	unsigned char product_name[128];
	ret = xrt_prober_get_string_descriptor( //
	    xp,                                 //
	    xpdev,                              //
	    XRT_PROBER_STRING_PRODUCT,          //
	    product_name,                       //
	    sizeof(product_name));              //
	if (ret <= 0) {
		U_LOG_E("Failed to get product name from Bluetooth device!");
		return NULL;
	}

	enum u_device_alloc_flags flags = U_DEVICE_ALLOC_TRACKING_NONE;
	struct pssense_device *pssense = U_DEVICE_ALLOCATE(struct pssense_device, flags, PSSENSE_INPUT_COUNT, 2);
	PSSENSE_DEBUG(pssense, "PlayStation Sense controller found");

	pssense->base.name = XRT_DEVICE_PSSENSE;
	snprintf(pssense->base.str, XRT_DEVICE_NAME_LEN, "%s", product_name);
	pssense->base.update_inputs = pssense_device_update_inputs;
	pssense->base.set_output = pssense_set_output;
	pssense->base.get_output_limits = pssense_get_output_limits;
	pssense->base.get_tracked_pose = pssense_get_tracked_pose;
	pssense->base.get_battery_status = pssense_get_battery_status;
	pssense->base.destroy = pssense_device_destroy;

	pssense->base.supported.orientation_tracking = true;
	pssense->base.supported.battery_status = true;

	pssense->base.binding_profiles = binding_profiles_pssense;
	pssense->base.binding_profile_count = ARRAY_SIZE(binding_profiles_pssense);

	m_imu_3dof_init(&pssense->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);

	pssense->log_level = debug_get_log_option_pssense_log();
	pssense->hid = hid;

	if (xpdev->product_id == PSSENSE_PID_LEFT) {
		pssense->base.device_type = XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
		pssense->hand = PSSENSE_HAND_LEFT;
	} else if (xpdev->product_id == PSSENSE_PID_RIGHT) {
		pssense->base.device_type = XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
		pssense->hand = PSSENSE_HAND_RIGHT;
	} else {
		PSSENSE_ERROR(pssense, "Unable to determine controller type");
		pssense_device_destroy(&pssense->base);
		return NULL;
	}

	SET_INPUT(PS_CLICK);
	SET_INPUT(SHARE_CLICK);
	SET_INPUT(OPTIONS_CLICK);
	SET_INPUT(SQUARE_CLICK);
	SET_INPUT(SQUARE_TOUCH);
	SET_INPUT(TRIANGLE_CLICK);
	SET_INPUT(TRIANGLE_TOUCH);
	SET_INPUT(CROSS_CLICK);
	SET_INPUT(CROSS_TOUCH);
	SET_INPUT(CIRCLE_CLICK);
	SET_INPUT(CIRCLE_TOUCH);
	SET_INPUT(SQUEEZE_CLICK);
	SET_INPUT(SQUEEZE_TOUCH);
	SET_INPUT(SQUEEZE_PROXIMITY);
	SET_INPUT(SQUEEZE_PROXIMITY_FLOAT);
	SET_INPUT(TRIGGER_CLICK);
	SET_INPUT(TRIGGER_TOUCH);
	SET_INPUT(TRIGGER_VALUE);
	SET_INPUT(TRIGGER_PROXIMITY);
	SET_INPUT(TRIGGER_PROXIMITY_FLOAT);
	SET_INPUT(THUMBSTICK);
	SET_INPUT(THUMBSTICK_CLICK);
	SET_INPUT(THUMBSTICK_TOUCH);
	SET_INPUT(GRIP_POSE);
	SET_INPUT(AIM_POSE);

	pssense->base.outputs[0].name = XRT_OUTPUT_NAME_PSSENSE_VIBRATION;
	pssense->base.outputs[1].name = XRT_OUTPUT_NAME_PSSENSE_TRIGGER_FEEDBACK;

	os_precise_sleeper_init(&pssense->sleeper);

	pssense->output.pcm_haptics_resampler = u_resampler_create(4000, PCM_SAMPLE_RATE);
	if (pssense->output.pcm_haptics_resampler == NULL) {
		PSSENSE_ERROR(pssense, "Failed to create PCM resampler");
		pssense_device_destroy(&pssense->base);
		return NULL;
	}

	ret = os_thread_helper_init(&pssense->controller_thread);
	if (ret != 0) {
		PSSENSE_ERROR(pssense, "Failed to init threading!");
		pssense_device_destroy(&pssense->base);
		return NULL;
	}

	ret = os_thread_helper_start(&pssense->controller_thread, pssense_run_thread, pssense);
	if (ret != 0) {
		PSSENSE_ERROR(pssense, "Failed to start thread!");
		pssense_device_destroy(&pssense->base);
		return NULL;
	}

	if (!pssense_get_calibration_data(pssense)) {
		PSSENSE_ERROR(pssense, "Failed to retrieve calibration data");
		pssense_device_destroy(&pssense->base);
		return NULL;
	}

	u_var_add_root(pssense, pssense->base.str, false);
	u_var_add_log_level(pssense, &pssense->log_level, "Log level");

	u_var_add_gui_header(pssense, &pssense->gui.button_states, "Button States");
	u_var_add_bool(pssense, &pssense->state.ps_click, "PS Click");
	if (pssense->hand == PSSENSE_HAND_LEFT) {
		u_var_add_bool(pssense, &pssense->state.share_click, "Share Click");
		u_var_add_bool(pssense, &pssense->state.square_click, "Square Click");
		u_var_add_bool(pssense, &pssense->state.square_touch, "Square Touch");
		u_var_add_bool(pssense, &pssense->state.triangle_click, "Triangle Click");
		u_var_add_bool(pssense, &pssense->state.triangle_touch, "Triangle Touch");
	} else if (pssense->hand == PSSENSE_HAND_RIGHT) {
		u_var_add_bool(pssense, &pssense->state.options_click, "Options Click");
		u_var_add_bool(pssense, &pssense->state.cross_click, "Cross Click");
		u_var_add_bool(pssense, &pssense->state.cross_touch, "Cross Touch");
		u_var_add_bool(pssense, &pssense->state.circle_click, "Circle Click");
		u_var_add_bool(pssense, &pssense->state.circle_touch, "Circle Touch");
	}
	u_var_add_bool(pssense, &pssense->state.squeeze_click, "Squeeze Click");
	u_var_add_bool(pssense, &pssense->state.squeeze_touch, "Squeeze Touch");
	u_var_add_ro_f32(pssense, &pssense->state.squeeze_proximity, "Squeeze Proximity");
	u_var_add_bool(pssense, &pssense->state.trigger_click, "Trigger Click");
	u_var_add_bool(pssense, &pssense->state.trigger_touch, "Trigger Touch");
	u_var_add_ro_f32(pssense, &pssense->state.trigger_value, "Trigger");
	u_var_add_ro_f32(pssense, &pssense->state.trigger_proximity, "Trigger Proximity");
	u_var_add_ro_f32(pssense, &pssense->state.thumbstick.x, "Thumbstick X");
	u_var_add_ro_f32(pssense, &pssense->state.thumbstick.y, "Thumbstick Y");
	u_var_add_bool(pssense, &pssense->state.thumbstick_click, "Thumbstick Click");
	u_var_add_bool(pssense, &pssense->state.thumbstick_touch, "Thumbstick Touch");

	u_var_add_gui_header(pssense, &pssense->gui.tracking, "Tracking");
	u_var_add_ro_vec3_i32(pssense, &pssense->state.gyro_raw, "Raw Gyro");
	u_var_add_ro_vec3_i32(pssense, &pssense->state.accel_raw, "Raw Accel");
	u_var_add_pose(pssense, &pssense->pose, "Pose");
	m_imu_3dof_add_vars(&pssense->fusion, pssense, "3dof Fusion");

	return &pssense->base;
}

int
pssense_found(struct xrt_prober *xp,
              struct xrt_prober_device **devices,
              size_t device_count,
              size_t index,
              cJSON *attached_data,
              struct xrt_device **out_xdevs)
{
	struct xrt_prober_device *xpdev = devices[index];

	struct xrt_device *xdev = pssense_create(xp, xpdev);
	if (xdev == NULL) {
		return -1;
	}

	out_xdevs[0] = xdev;
	return 1;
}

/*!
 * @}
 */
