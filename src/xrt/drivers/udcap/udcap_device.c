// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  UDCAP glove device: an Index-style controller with finger skeletal
 *         tracking, sourced from the udcap-server shared memory.
 *
 * The device presents as a left/right hand controller (so its A/B/trigger/grip/
 * joystick bind through the Valve Index interaction profile) and also provides
 * articulated hand tracking from the glove's per-bone quaternions. Its 6DoF pose
 * comes from a Lighthouse tracker via a tracking override (see udcap_prober.c),
 * which supplies grip/aim pose and places the hand joints.
 *
 * @ingroup drv_udcap
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "xrt/xrt_device.h"
#include "xrt/xrt_defines.h"

#include "math/m_api.h"
#include "math/m_space.h"

#include "util/u_device.h"
#include "util/u_debug.h"
#include "util/u_hand_tracking.h"
#include "util/u_hand_simulation.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_var.h"

#include "os/os_time.h"

#include "udcap_device.h"
#include "udcap_shm.h"

DEBUG_GET_ONCE_LOG_OPTION(udcap_log, "UDCAP_LOG", U_LOGGING_INFO)

// Bone bend angle (rad) mapped to curl=1.0. Tunable; refined against VR feel.
#define UDCAP_MAX_BEND_RAD 1.5f
// Analog trigger value above which the trigger "click" boolean is asserted.
#define UDCAP_TRIGGER_CLICK_THRESHOLD 0.7f

enum udcap_input_index
{
	UDCAP_INPUT_HAND_TRACKING = 0,
	UDCAP_INPUT_GRIP_POSE,
	UDCAP_INPUT_AIM_POSE,
	UDCAP_INPUT_SYSTEM_CLICK,
	UDCAP_INPUT_A_CLICK,
	UDCAP_INPUT_B_CLICK,
	UDCAP_INPUT_TRIGGER_VALUE,
	UDCAP_INPUT_TRIGGER_CLICK,
	UDCAP_INPUT_SQUEEZE_VALUE,
	UDCAP_INPUT_THUMBSTICK,
	UDCAP_INPUT_THUMBSTICK_CLICK,
	UDCAP_INPUT_COUNT
};

/*!
 * @implements xrt_device
 */
struct udcap_device
{
	struct xrt_device base;

	enum xrt_hand hand;
	int hand_index; // UDCAP_HAND_LEFT / RIGHT

	struct xrt_device *tracker; // borrowed pose source (Lighthouse tracker)

	int shm_fd;
	udcap_shm *shm;

	enum u_logging_level log_level;
};

#define UDCAP_DEG_TO_RAD(d) ((float)(d) * (float)(M_PI / 180.0))

static inline struct udcap_device *
udcap_device(struct xrt_device *xdev)
{
	return (struct udcap_device *)xdev;
}

// Consistent snapshot of this hand's shm slot (server-written payload only).
static void
udcap_snapshot(struct udcap_device *ud, udcap_hand *out)
{
	const udcap_hand *H = &ud->shm->hands[ud->hand_index];
	uint32_t s;
	do {
		s = udcap_read_begin(H);
		*out = *H;
	} while (udcap_read_retry(H, s));
}

static float
bone_curl(udcap_quat q)
{
	float w = fabsf(q.w);
	if (w > 1.0f) {
		w = 1.0f;
	}
	float angle = 2.0f * acosf(w);
	float c = angle / UDCAP_MAX_BEND_RAD;
	if (c < 0.0f) {
		c = 0.0f;
	}
	if (c > 1.0f) {
		c = 1.0f;
	}
	return c;
}

// Per-finger user curl range: c in [lo,hi] -> [0,1]. Degenerate range = passthrough.
static float
remap_curl(float c, float lo, float hi)
{
	if (hi - lo < 0.01f) {
		return c;
	}
	float r = (c - lo) / (hi - lo);
	if (r < 0.0f) {
		r = 0.0f;
	}
	if (r > 1.0f) {
		r = 1.0f;
	}
	return r;
}

// Map the core's three measured bone bends onto Monado's hand-sim joint_curls.
// The sim's generic finger transform consumes joint_curls as:
//   [0] metacarpal (palm bone, ~fixed in a curl) | [1] proximal/MCP knuckle |
//   [2] intermediate/PIP | [3] distal/DIP.
// The thumb path is different: [0] metacarpal swing, [1]/[2] the two rotations.
static void
fill_finger(struct u_hand_tracking_finger_value *dst, const udcap_finger *f, int joint_count, float lo, float hi,
            bool is_thumb)
{
	float prox = bone_curl(f->proximal);
	float inter = bone_curl(f->intermediate);
	float dist = bone_curl(f->distal);

	dst->splay = 0.0f; // TODO: derive splay from adduction calibration
	dst->joint_count = joint_count;

	if (is_thumb) {
		dst->joint_curls[0] = remap_curl(prox, lo, hi);
		dst->joint_curls[1] = remap_curl(inter, lo, hi);
		dst->joint_curls[2] = remap_curl(dist, lo, hi);
		dst->joint_curls[3] = 0.0f;
		return;
	}

	// The fingertip (DIP) sensor reads weakest; let it follow the middle joint
	// a little so the tip doesn't stay rigidly straight.
	if (dist < inter * 0.5f) {
		dist = inter * 0.5f;
	}

	dst->joint_curls[0] = 0.0f;                      // metacarpal: not measured
	dst->joint_curls[1] = remap_curl(prox, lo, hi);  // MCP (first phalanx)
	dst->joint_curls[2] = remap_curl(inter, lo, hi); // PIP (middle phalanx)
	dst->joint_curls[3] = remap_curl(dist, lo, hi);  // DIP (fingertip)
}

// The per-hand offset, read live from shm. Position is applied directly in the
// tracker's frame (intuitive to tune), while the rotation uses the inverse of
// the euler angles — the orientation convention that matched UDCAP's values.
// The hand-tracking root and the OpenXR grip pose use opposite orientation
// conventions: the avatar hand looks right with the offset orientation inverted
// (conjugate), while the grip pose (which VRChat anchors its menu to) wants the
// offset applied directly, like opengloves-driver does. So `invert` selects.
static void
udcap_offset_pose(struct udcap_device *ud, struct xrt_pose *out, bool invert)
{
	const udcap_hand *H = &ud->shm->hands[ud->hand_index];
	out->position.x = H->offset_pos[0];
	out->position.y = H->offset_pos[1];
	out->position.z = H->offset_pos[2];
	struct xrt_vec3 euler = {UDCAP_DEG_TO_RAD(H->offset_rot_deg[0]), UDCAP_DEG_TO_RAD(H->offset_rot_deg[1]),
	                         UDCAP_DEG_TO_RAD(H->offset_rot_deg[2])};
	math_quat_from_euler_angles(&euler, &out->orientation);
	if (invert) {
		// Conjugate of a unit quat.
		out->orientation.x = -out->orientation.x;
		out->orientation.y = -out->orientation.y;
		out->orientation.z = -out->orientation.z;
	}
}

// Controller pose = tracker pose with the live offset applied directly:
// controller = tracker ∘ offset (offset is the grip pose in the tracker frame).
static void
udcap_compose_pose(struct udcap_device *ud, int64_t at_timestamp_ns, struct xrt_space_relation *out_rel, bool invert)
{
	if (ud->tracker == NULL) {
		m_space_relation_ident(out_rel);
		out_rel->relation_flags = (enum xrt_space_relation_flags)(
		    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
		return;
	}

	struct xrt_space_relation tracker_rel;
	xrt_result_t xret =
	    xrt_device_get_tracked_pose(ud->tracker, XRT_INPUT_GENERIC_TRACKER_POSE, at_timestamp_ns, &tracker_rel);
	if (xret != XRT_SUCCESS) {
		m_space_relation_ident(out_rel);
		return;
	}

	struct xrt_pose offset;
	udcap_offset_pose(ud, &offset, invert);

	struct xrt_relation_chain xrc = {0};
	m_relation_chain_push_pose_if_not_identity(&xrc, &offset);
	m_relation_chain_push_relation(&xrc, &tracker_rel);
	m_relation_chain_resolve(&xrc, out_rel);
}

static xrt_result_t
udcap_device_get_hand_tracking(struct xrt_device *xdev,
                               enum xrt_input_name name,
                               int64_t requested_timestamp_ns,
                               struct xrt_hand_joint_set *out_joint_set,
                               int64_t *out_timestamp_ns)
{
	struct udcap_device *ud = udcap_device(xdev);

	udcap_hand snap;
	udcap_snapshot(ud, &snap);

	*out_timestamp_ns = requested_timestamp_ns;

	if (!snap.present || !snap.calibrated) {
		out_joint_set->is_active = false;
		return XRT_SUCCESS;
	}

	// Curl ranges are indexed [thumb, index, middle, ring, little].
	struct u_hand_tracking_values values = {0};
	fill_finger(&values.little, &snap.skel.little, 5, snap.curl_min[4], snap.curl_max[4], false);
	fill_finger(&values.ring, &snap.skel.ring, 5, snap.curl_min[3], snap.curl_max[3], false);
	fill_finger(&values.middle, &snap.skel.middle, 5, snap.curl_min[2], snap.curl_max[2], false);
	fill_finger(&values.index, &snap.skel.index, 5, snap.curl_min[1], snap.curl_max[1], false);
	fill_finger(&values.thumb, &snap.skel.thumb, 4, snap.curl_min[0], snap.curl_max[0], true);

	struct xrt_space_relation ident;
	m_space_relation_ident(&ident);
	u_hand_sim_simulate_generic(&values, ud->hand, &ident, out_joint_set);

	// Place the hand at the tracker pose (+ live offset).
	udcap_compose_pose(ud, requested_timestamp_ns, &out_joint_set->hand_pose, true);

	out_joint_set->is_active = true;
	return XRT_SUCCESS;
}

static xrt_result_t
udcap_device_get_tracked_pose(struct xrt_device *xdev,
                              enum xrt_input_name name,
                              int64_t at_timestamp_ns,
                              struct xrt_space_relation *out_relation)
{
	(void)name; // grip and aim share the same pose
	// Grip uses the offset applied directly (opengloves convention) so VRChat's
	// menu anchors to the palm side, not the inverted hand-tracking orientation.
	udcap_compose_pose(udcap_device(xdev), at_timestamp_ns, out_relation, false);
	return XRT_SUCCESS;
}

static xrt_result_t
udcap_device_update_inputs(struct xrt_device *xdev)
{
	struct udcap_device *ud = udcap_device(xdev);

	udcap_hand snap;
	udcap_snapshot(ud, &snap);

	int64_t now = os_monotonic_get_ns();
	struct xrt_input *in = ud->base.inputs;
	in[UDCAP_INPUT_SYSTEM_CLICK].value.boolean = snap.btn_menu != 0;
	in[UDCAP_INPUT_A_CLICK].value.boolean = snap.btn_a != 0;
	in[UDCAP_INPUT_B_CLICK].value.boolean = snap.btn_b != 0;
	in[UDCAP_INPUT_TRIGGER_VALUE].value.vec1.x = snap.trigger;
	in[UDCAP_INPUT_TRIGGER_CLICK].value.boolean = snap.trigger >= UDCAP_TRIGGER_CLICK_THRESHOLD;
	in[UDCAP_INPUT_SQUEEZE_VALUE].value.vec1.x = snap.grip;
	in[UDCAP_INPUT_THUMBSTICK].value.vec2.x = snap.joy_x;
	in[UDCAP_INPUT_THUMBSTICK].value.vec2.y = snap.joy_y;
	in[UDCAP_INPUT_THUMBSTICK_CLICK].value.boolean = snap.btn_joy != 0;

	for (size_t i = 0; i < UDCAP_INPUT_COUNT; i++) {
		in[i].timestamp = now;
	}

	return XRT_SUCCESS;
}

static xrt_result_t
udcap_device_set_output(struct xrt_device *xdev, enum xrt_output_name name, const struct xrt_output_value *value)
{
	struct udcap_device *ud = udcap_device(xdev);
	udcap_hand *H = &ud->shm->hands[ud->hand_index];

	float dur = (float)value->vibration.duration_ns / 1.0e9f;
	if (dur < 0.05f) {
		dur = 0.05f;
	}
	if (dur > 5.0f) {
		dur = 5.0f;
	}
	int strength = (int)(value->vibration.amplitude * 200.0f);
	if (strength < 0) {
		strength = 0;
	}

	// Write params first, then publish via the request counter (release).
	H->haptic_index = -1;
	H->haptic_duration_s = dur;
	H->haptic_strength = strength;
	__atomic_add_fetch(&H->haptic_seq, 1, __ATOMIC_RELEASE);

	return XRT_SUCCESS;
}

static void
udcap_device_destroy(struct xrt_device *xdev)
{
	struct udcap_device *ud = udcap_device(xdev);

	u_var_remove_root(ud);

	if (ud->shm != NULL && ud->shm != MAP_FAILED) {
		munmap(ud->shm, sizeof(udcap_shm));
	}
	if (ud->shm_fd >= 0) {
		close(ud->shm_fd);
	}

	u_device_free(&ud->base);
}

struct xrt_device *
udcap_device_create(enum xrt_hand hand)
{
	enum u_logging_level log_level = debug_get_log_option_udcap_log();

	// Open the shared memory published by udcap-server.
	int fd = shm_open(UDCAP_SHM_NAME, O_RDWR, 0);
	if (fd < 0) {
		U_LOG_IFL_I(log_level, "udcap: shm %s not available (is udcap-server running?)", UDCAP_SHM_NAME);
		return NULL;
	}
	udcap_shm *shm = mmap(NULL, sizeof(udcap_shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (shm == MAP_FAILED) {
		U_LOG_IFL_E(log_level, "udcap: mmap failed");
		close(fd);
		return NULL;
	}
	if (shm->magic != UDCAP_SHM_MAGIC || shm->version != UDCAP_SHM_VERSION) {
		U_LOG_IFL_E(log_level, "udcap: shm magic/version mismatch (got %08x/%u)", shm->magic, shm->version);
		munmap(shm, sizeof(udcap_shm));
		close(fd);
		return NULL;
	}

	enum u_device_alloc_flags flags = (enum u_device_alloc_flags)(U_DEVICE_ALLOC_TRACKING_NONE);
	struct udcap_device *ud = U_DEVICE_ALLOCATE(struct udcap_device, flags, UDCAP_INPUT_COUNT, 1);

	ud->hand = hand;
	ud->hand_index = (hand == XRT_HAND_LEFT) ? UDCAP_HAND_LEFT : UDCAP_HAND_RIGHT;
	ud->shm_fd = fd;
	ud->shm = shm;
	ud->log_level = log_level;

	// Present as an Index controller so the Valve Index interaction profile
	// binds the buttons/stick; also a hand-tracking source.
	ud->base.name = XRT_DEVICE_INDEX_CONTROLLER;
	ud->base.device_type =
	    (hand == XRT_HAND_LEFT) ? XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER : XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;

	ud->base.get_hand_tracking = udcap_device_get_hand_tracking;
	ud->base.get_tracked_pose = udcap_device_get_tracked_pose;
	ud->base.update_inputs = udcap_device_update_inputs;
	ud->base.set_output = udcap_device_set_output;
	ud->base.destroy = udcap_device_destroy;

	ud->base.supported.hand_tracking = true;
	ud->base.supported.orientation_tracking = true;
	ud->base.supported.position_tracking = true;

	ud->base.inputs[UDCAP_INPUT_HAND_TRACKING].name =
	    hand == XRT_HAND_LEFT ? XRT_INPUT_HT_UNOBSTRUCTED_LEFT : XRT_INPUT_HT_UNOBSTRUCTED_RIGHT;
	ud->base.inputs[UDCAP_INPUT_GRIP_POSE].name = XRT_INPUT_INDEX_GRIP_POSE;
	ud->base.inputs[UDCAP_INPUT_AIM_POSE].name = XRT_INPUT_INDEX_AIM_POSE;
	ud->base.inputs[UDCAP_INPUT_SYSTEM_CLICK].name = XRT_INPUT_INDEX_SYSTEM_CLICK;
	ud->base.inputs[UDCAP_INPUT_A_CLICK].name = XRT_INPUT_INDEX_A_CLICK;
	ud->base.inputs[UDCAP_INPUT_B_CLICK].name = XRT_INPUT_INDEX_B_CLICK;
	ud->base.inputs[UDCAP_INPUT_TRIGGER_VALUE].name = XRT_INPUT_INDEX_TRIGGER_VALUE;
	ud->base.inputs[UDCAP_INPUT_TRIGGER_CLICK].name = XRT_INPUT_INDEX_TRIGGER_CLICK;
	ud->base.inputs[UDCAP_INPUT_SQUEEZE_VALUE].name = XRT_INPUT_INDEX_SQUEEZE_VALUE;
	ud->base.inputs[UDCAP_INPUT_THUMBSTICK].name = XRT_INPUT_INDEX_THUMBSTICK;
	ud->base.inputs[UDCAP_INPUT_THUMBSTICK_CLICK].name = XRT_INPUT_INDEX_THUMBSTICK_CLICK;

	ud->base.outputs[0].name = XRT_OUTPUT_NAME_INDEX_HAPTIC;

	snprintf(ud->base.str, XRT_DEVICE_NAME_LEN, "UDCAP Glove %s", hand == XRT_HAND_LEFT ? "Left" : "Right");
	snprintf(ud->base.serial, XRT_DEVICE_NAME_LEN, "UDCAP %s", hand == XRT_HAND_LEFT ? "Left" : "Right");

	u_var_add_root(ud, ud->base.str, true);

	U_LOG_IFL_I(log_level, "udcap: created %s controller (with hand tracking)",
	            hand == XRT_HAND_LEFT ? "left" : "right");

	return &ud->base;
}

void
udcap_device_set_tracker(struct xrt_device *xdev, struct xrt_device *tracker)
{
	struct udcap_device *ud = udcap_device(xdev);
	ud->tracker = tracker;
	if (tracker != NULL) {
		ud->base.tracking_origin = tracker->tracking_origin;
	}
}
