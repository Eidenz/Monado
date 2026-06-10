// SPDX-License-Identifier: MIT
/*!
 * @file udcap_shm.h
 * @brief Shared-memory contract between the UDCAP bridge server and consumers
 *        (the Monado drv_udcap driver, and any GUI such as a future Tauri app).
 *
 * One writer per hand (the server) and any number of readers. Each hand slot is
 * protected by a seqlock so readers get a consistent snapshot without locking.
 * Haptics flow the other way: a reader (the driver) bumps `haptic_seq` to
 * request a buzz; the server polls it.
 *
 * This header is plain C and is included verbatim by both the C++ server and the
 * C Monado driver. Keep it dependency-free and in sync across both trees; the
 * `version` field guards against layout mismatches at runtime.
 */
#ifndef UDCAP_SHM_H
#define UDCAP_SHM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* shm_open() name (lives at /dev/shm/udcap_hands on Linux). */
#define UDCAP_SHM_NAME "/udcap_hands"
#define UDCAP_SHM_MAGIC 0x55444331u /* "UDC1" */
#define UDCAP_SHM_VERSION 11u

enum udcap_hand_index
{
	UDCAP_HAND_LEFT = 0,
	UDCAP_HAND_RIGHT = 1,
	UDCAP_HAND_COUNT = 2
};

/* Link state mirrors the core's UdState. */
enum udcap_link_state
{
	UDCAP_LINK_INIT = 0,
	UDCAP_LINK_NOT_CONNECTED = 1,
	UDCAP_LINK_CONNECTED = 2,
	UDCAP_LINK_LINKED = 3
};

/* Commands issued by a control app (GUI) to the server. The app writes cmd_code
 * (+cmd_arg) then bumps cmd_seq; the server executes and echoes cmd_ack=cmd_seq.
 * The guided calibration is: START -> FIST -> TOGETHER -> SPREAD -> COMPLETE. */
enum udcap_cmd
{
	UDCAP_CMD_NONE = 0,
	UDCAP_CMD_CALIB_START = 1,    /* runCalibration on both hands         */
	UDCAP_CMD_CALIB_FIST = 2,     /* capture "lightly make a fist"        */
	UDCAP_CMD_CALIB_TOGETHER = 3, /* capture "close five fingers together"*/
	UDCAP_CMD_CALIB_SPREAD = 4,   /* capture "spread out five fingers"    */
	UDCAP_CMD_CALIB_COMPLETE = 5, /* finalize calibration                 */
	UDCAP_CMD_CALIB_CANCEL = 6,
	UDCAP_CMD_CALIB_AUTO = 7 /* run the whole timed fist/together/spread sequence */
};

/* Calibration progress, reported by the server. */
enum udcap_calib_state
{
	UDCAP_CALIB_IDLE = 0,
	UDCAP_CALIB_STARTED = 1,      /* ready for the fist pose */
	UDCAP_CALIB_GOT_FIST = 2,
	UDCAP_CALIB_GOT_TOGETHER = 3,
	UDCAP_CALIB_GOT_SPREAD = 4,
	UDCAP_CALIB_DONE = 5,
	UDCAP_CALIB_ERROR = 6,
	UDCAP_CALIB_READY = 7 /* "get ready" countdown before the first pose */
};

/* Button remap: which physical glove input drives each controller button.
 * Indexed by udcap_btn_out, valued by udcap_btn_src (see btn_src below). */
enum udcap_btn_out
{
	UDCAP_OUT_A = 0,       /* A click */
	UDCAP_OUT_B = 1,       /* B click */
	UDCAP_OUT_SYSTEM = 2,  /* system / menu click */
	UDCAP_OUT_STICK = 3,   /* thumbstick click */
	UDCAP_OUT_TRIGGER = 4, /* force trigger to full when source is pressed */
	UDCAP_OUT_GRIP = 5,    /* force grip to full when source is pressed */
	UDCAP_OUT_COUNT = 6
};

/* Which finger drives the analog trigger/grip (index into [thumb,index,middle,
 * ring,little]); grip also accepts UDCAP_FINGER_GRIP = average of M+R+P. */
enum udcap_finger_sel
{
	UDCAP_FINGER_GRIP = 5
};
enum udcap_btn_src
{
	UDCAP_SRC_NONE = 0,
	UDCAP_SRC_A = 1,
	UDCAP_SRC_B = 2,
	UDCAP_SRC_MENU = 3, /* A + B pressed together */
	UDCAP_SRC_STICK = 4 /* thumbstick click */
};

typedef struct udcap_quat
{
	float x, y, z, w;
} udcap_quat;

/* Per-finger bone orientations, as emitted by the core's CMD_SKELETON_QUATERNION. */
typedef struct udcap_finger
{
	udcap_quat proximal;
	udcap_quat intermediate;
	udcap_quat distal;
} udcap_finger;

typedef struct udcap_skeleton
{
	udcap_finger thumb;
	udcap_finger index;
	udcap_finger middle;
	udcap_finger ring;
	udcap_finger little;
} udcap_skeleton;

/*
 * One hand. Writer (server) brackets updates with seq odd->even; readers spin
 * on `seq` (see helpers below). Everything after `seq` is the payload.
 */
typedef struct udcap_hand
{
	uint32_t seq;          /* seqlock: even = stable, odd = being written */
	uint32_t present;      /* 1 if a glove for this hand is enumerated     */
	uint32_t link;         /* enum udcap_link_state                        */
	uint32_t calibrated;   /* 1 once calibration is complete (skel valid)  */
	uint32_t battery;      /* battery reading from the core                */
	uint32_t _pad0;
	uint64_t timestamp_ns; /* CLOCK_MONOTONIC ns when the server wrote this */

	udcap_skeleton skel; /* computed per-bone quaternions */

	/* Index-finger controller module. */
	float joy_x, joy_y;            /* -1..1 */
	float trigger, grip, trackpad; /* virtual analog, 0..1 */
	uint32_t btn_a, btn_b, btn_menu, btn_joy, btn_power;

	/* Identity + link quality (for the control app). */
	float fps;              /* packets/sec for this hand */
	char fw[16];            /* glove firmware version */
	char glove_serial[24];  /* glove serial number */

	/* Haptics request (reader -> server). Reader bumps `haptic_seq`; server
	 * sends mcuSendVibration(index, duration_s, strength) when it changes. */
	uint32_t haptic_seq;
	int32_t haptic_index;    /* actuator index, -1 = default */
	float haptic_duration_s; /* seconds */
	int32_t haptic_strength; /* core strength units */

	/* Config (NOT in the seqlock payload). Written by the server at startup and
	 * tunable live (e.g. by udcap-offset or a GUI); the driver reads these every
	 * frame, so offset changes take effect without restarting VR. */
	char tracker_serial[32]; /* Lighthouse tracker to attach pose to ("" = fallback) */

	/* Pose offset = physical pose of the TRACKER in the controller's frame
	 * (same meaning as Monado's tracking-override offset and UDCAP's "Space
	 * Orientation" offsets). */
	float offset_pos[3];     /* meters: x, y, z */
	float offset_rot_deg[3]; /* euler degrees: x, y, z */

	/* Per-finger curl remap, applied by the driver after the core's curl:
	 * out = clamp((curl - min) / (max - min), 0, 1). Index order is
	 * [thumb, index, middle, ring, little]. Defaults: min 0, max 1 (identity). */
	float curl_min[5];
	float curl_max[5];

	/* Extra position (meters, tracker frame) + rotation (degrees, hand-local)
	 * applied to the GRIP/aim pose only, on top of the hand offset. Corrects the
	 * OpenXR grip convention so VRChat's menu sits on the palm side. The
	 * hand-tracking pose ignores these. */
	float grip_pos[3];
	float grip_rot_deg[3];

	/* Per-hand input mapping. btn_src[udcap_btn_out] = udcap_btn_src; the analog
	 * trigger/grip are computed from the chosen finger's curl, remapped through
	 * [min,max]. */
	uint8_t btn_src[UDCAP_OUT_COUNT];
	uint8_t trigger_finger; /* 0..4 */
	uint8_t grip_finger;    /* 0..4, or UDCAP_FINGER_GRIP (avg M+R+P) */
	float trigger_min, trigger_max;
	float grip_min, grip_max;
} udcap_hand;

typedef struct udcap_shm
{
	uint32_t magic;      /* UDCAP_SHM_MAGIC */
	uint32_t version;    /* UDCAP_SHM_VERSION */
	uint32_t server_pid; /* pid of the writing server (0 = none) */
	uint32_t _pad;
	udcap_hand hands[UDCAP_HAND_COUNT];

	/* Control-app -> server command channel. The app sets cmd_code (+cmd_arg)
	 * then bumps cmd_seq; the server executes and sets cmd_ack = cmd_seq. */
	uint32_t cmd_seq;
	uint32_t cmd_code; /* enum udcap_cmd */
	int32_t cmd_arg;
	uint32_t cmd_ack;

	/* Calibration progress (enum udcap_calib_state), written by the server. */
	uint32_t calib_state;

	/* Global finger-curl strength: scales the curl the driver feeds the hand
	 * sim. 1.5 = full fist (default/max); lower closes the hand less. */
	float curl_gain;

	/* Global finger-splay (abduction) scale. 1 = measured, negative flips the
	 * spread direction, 0 = off. */
	float splay_gain;
} udcap_shm;

/*
 * Seqlock helpers. Implemented with GCC/Clang __atomic builtins so the one
 * header works unchanged in both the C driver and the C++ server.
 */

/* Writer: call before/after mutating a hand's payload. */
static inline void
udcap_write_begin(udcap_hand *h)
{
	__atomic_store_n(&h->seq, h->seq + 1u, __ATOMIC_RELAXED);
	__atomic_thread_fence(__ATOMIC_RELEASE);
}
static inline void
udcap_write_end(udcap_hand *h)
{
	__atomic_thread_fence(__ATOMIC_RELEASE);
	__atomic_store_n(&h->seq, h->seq + 1u, __ATOMIC_RELAXED);
}

/* Reader: snapshot pattern.
 *   uint32_t s;
 *   do { s = udcap_read_begin(h); copy = *h; } while (udcap_read_retry(h, s));
 */
static inline uint32_t
udcap_read_begin(const udcap_hand *h)
{
	uint32_t s;
	do {
		s = __atomic_load_n(&h->seq, __ATOMIC_ACQUIRE);
	} while (s & 1u);
	return s;
}
static inline int
udcap_read_retry(const udcap_hand *h, uint32_t start_seq)
{
	__atomic_thread_fence(__ATOMIC_ACQUIRE);
	return __atomic_load_n(&h->seq, __ATOMIC_RELAXED) != start_seq;
}

#ifdef __cplusplus
}
#endif

#endif /* UDCAP_SHM_H */
