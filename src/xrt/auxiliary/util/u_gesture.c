// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Server-side hand-gesture detector.
 * @ingroup aux_util
 */

#include "util/u_gesture.h"
#include "util/u_misc.h"
#include "util/u_logging.h"
#include "util/u_screenshot.h"
#include "util/u_gesture_feedback.h"
#include "util/u_file.h"
#include "util/u_json.h"

#include "xrt/xrt_device.h"
#include "xrt/xrt_system.h"
#include "xrt/xrt_defines.h"

#include "math/m_api.h"

#include "os/os_threading.h"
#include "os/os_time.h"

#include <stdlib.h>
#include <math.h>
#include <sys/stat.h>


/*
 *
 * Tunables (will move to a config file in a later milestone).
 *
 */

#define TICK_NS (1000000000LL / 90)   // ~90 Hz poll
#define DEFAULT_HOLD_MS 2000          // pose must be held this long to fire
#define COOLDOWN_MS 1500              // ignore re-fires for this long after firing

// A finger is "extended" if the cosine of the bend angle is above this, and
// "curled" if below the lower one. The gap avoids boundary flicker.
#define EXT_COS 0.45f
#define CURL_COS 0.35f

// "Forms a frame" gate: the two hands' index fingers must point roughly
// opposite ways (cos below this), and likewise the thumbs. This is what
// distinguishes an actual frame from two same-direction L-poses.
#define FRAME_OPP_COS -0.40f

// Two-hand proximity gate: hands must be within this range (metres) apart.
// Generous for now (pose + hold is the real gate); tighten from logs later.
#define MIN_WRIST_DIST 0.02f
#define MAX_WRIST_DIST 1.50f


/*
 *
 * Detector state.
 *
 */

struct u_gesture
{
	struct os_thread_helper oth;
	struct xrt_system_devices *xsysd; // borrowed

	bool enabled;
	bool debug;
	bool frame_feedback; // draw the in-headset rectangle while the pose is held
	int64_t hold_ns;
	int64_t cooldown_ns;

	// State machine.
	int64_t hold_start_ns; // 0 = not currently in pose
	int64_t last_fire_ns;
	int64_t last_debug_ns;

	// Config file (XDG) + hot-reload tracking.
	char config_path[512];
	int64_t config_mtime;
	int64_t last_config_check_ns;
};


/*
 *
 * Small vec3 helpers (avoid pulling in more deps for a handful of ops).
 *
 */

static inline struct xrt_vec3
v_sub(struct xrt_vec3 a, struct xrt_vec3 b)
{
	return (struct xrt_vec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline float
v_dot(struct xrt_vec3 a, struct xrt_vec3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline float
v_len(struct xrt_vec3 a)
{
	return sqrtf(v_dot(a, a));
}

static inline struct xrt_vec3
joint_pos(const struct xrt_hand_joint_set *s, enum xrt_hand_joint j)
{
	return s->values.hand_joint_set_default[j].relation.pose.position;
}

/*!
 * Cosine of the angle between the (base->mid) and (mid->tip) bone directions.
 * ~1 when the finger is straight/extended, drops toward 0 and negative as it
 * curls toward the palm.
 */
static float
finger_straightness(const struct xrt_hand_joint_set *s,
                    enum xrt_hand_joint base,
                    enum xrt_hand_joint mid,
                    enum xrt_hand_joint tip)
{
	struct xrt_vec3 a = v_sub(joint_pos(s, mid), joint_pos(s, base));
	struct xrt_vec3 b = v_sub(joint_pos(s, tip), joint_pos(s, mid));
	float la = v_len(a), lb = v_len(b);
	if (la < 1e-6f || lb < 1e-6f) {
		return 0.0f;
	}
	return v_dot(a, b) / (la * lb);
}

// World-space unit direction of a finger segment (base->tip). Joint positions
// are hand-local, so rotate the local delta by the hand's world orientation.
static struct xrt_vec3
world_finger_dir(const struct xrt_hand_joint_set *s, enum xrt_hand_joint base, enum xrt_hand_joint tip)
{
	struct xrt_vec3 local = v_sub(joint_pos(s, tip), joint_pos(s, base));
	struct xrt_vec3 world;
	math_quat_rotate_vec3(&s->hand_pose.pose.orientation, &local, &world);
	float len = v_len(world);
	if (len > 1e-6f) {
		world.x /= len;
		world.y /= len;
		world.z /= len;
	}
	return world;
}


/*
 *
 * Pose detection.
 *
 */

struct hand_pose
{
	float thumb, index, middle, ring, little;
	bool is_frame; // thumb+index extended, other three curled
};

static struct hand_pose
analyse_hand(const struct xrt_hand_joint_set *s)
{
	struct hand_pose p;
	p.thumb = finger_straightness(s, XRT_HAND_JOINT_THUMB_METACARPAL, XRT_HAND_JOINT_THUMB_PROXIMAL,
	                              XRT_HAND_JOINT_THUMB_TIP);
	p.index = finger_straightness(s, XRT_HAND_JOINT_INDEX_METACARPAL, XRT_HAND_JOINT_INDEX_PROXIMAL,
	                              XRT_HAND_JOINT_INDEX_TIP);
	p.middle = finger_straightness(s, XRT_HAND_JOINT_MIDDLE_METACARPAL, XRT_HAND_JOINT_MIDDLE_PROXIMAL,
	                               XRT_HAND_JOINT_MIDDLE_TIP);
	p.ring = finger_straightness(s, XRT_HAND_JOINT_RING_METACARPAL, XRT_HAND_JOINT_RING_PROXIMAL,
	                             XRT_HAND_JOINT_RING_TIP);
	p.little = finger_straightness(s, XRT_HAND_JOINT_LITTLE_METACARPAL, XRT_HAND_JOINT_LITTLE_PROXIMAL,
	                               XRT_HAND_JOINT_LITTLE_TIP);

	p.is_frame = p.thumb > EXT_COS &&    //
	             p.index > EXT_COS &&     //
	             p.middle < CURL_COS &&   //
	             p.ring < CURL_COS &&     //
	             p.little < CURL_COS;
	return p;
}


/*
 *
 * Device resolution.
 *
 */

// Find the device's hand-tracking input name (gloves use UNOBSTRUCTED,
// controllers CONFORMING; scanning works for both).
static bool
find_ht_input(struct xrt_device *xdev, enum xrt_input_name *out_name)
{
	if (xdev == NULL || xdev->get_hand_tracking == NULL) {
		return false;
	}
	for (size_t i = 0; i < xdev->input_count; i++) {
		enum xrt_input_name name = xdev->inputs[i].name;
		if (XRT_GET_INPUT_TYPE(name) == XRT_INPUT_TYPE_HAND_TRACKING) {
			*out_name = name;
			return true;
		}
	}
	return false;
}

static bool
get_hand(struct xrt_system_devices *xsysd, int32_t role_idx, int64_t now_ns, struct xrt_hand_joint_set *out)
{
	if (role_idx < 0 || (size_t)role_idx >= xsysd->static_xdev_count) {
		return false;
	}
	struct xrt_device *xdev = xsysd->static_xdevs[role_idx];
	enum xrt_input_name ht_name = 0;
	if (!find_ht_input(xdev, &ht_name)) {
		return false;
	}

	int64_t out_ts = 0;
	xrt_result_t r = xrt_device_get_hand_tracking(xdev, ht_name, now_ns, out, &out_ts);
	return r == XRT_SUCCESS && out->is_active;
}


/*
 *
 * Framed-crop projection.
 *
 */

// Project a world point into the eye's normalised image coords [0,1], top-left
// origin (Vulkan Y-down). Returns false if the point is behind the eye.
static bool
project_to_uv(const struct xrt_matrix_4x4 *view, const struct xrt_fov *fov, struct xrt_vec3 world, float *out_u,
              float *out_v)
{
	struct xrt_vec3 cam;
	math_matrix_4x4_transform_vec3(view, &world, &cam); // view is rigid, so w=1
	float depth = -cam.z;                               // eye looks down -Z
	if (depth <= 0.001f) {
		return false;
	}
	float tx = cam.x / depth;
	float ty = cam.y / depth;
	float tl = tanf(fov->angle_left);
	float tr = tanf(fov->angle_right);
	float tu = tanf(fov->angle_up);
	float td = tanf(fov->angle_down);
	if (tr - tl < 1e-6f || tu - td < 1e-6f) {
		return false;
	}
	*out_u = (tx - tl) / (tr - tl);
	*out_v = (tu - ty) / (tu - td); // angle_up -> 0 (top), angle_down -> 1 (bottom)
	return true;
}

// The frame's corners: thumb tip + index tip of each hand, in tracking space.
// Order is irrelevant downstream (both the crop and the overlay take a bounding
// box of the four points).
static void
compute_frame_world_corners(const struct xrt_hand_joint_set *left,
                            const struct xrt_hand_joint_set *right,
                            struct xrt_vec3 out_corners[4])
{
	const struct xrt_hand_joint_set *sets[4] = {left, left, right, right};
	const enum xrt_hand_joint js[4] = {XRT_HAND_JOINT_THUMB_TIP, XRT_HAND_JOINT_INDEX_TIP,
	                                   XRT_HAND_JOINT_THUMB_TIP, XRT_HAND_JOINT_INDEX_TIP};
	for (int i = 0; i < 4; i++) {
		struct xrt_vec3 local = joint_pos(sets[i], js[i]);
		math_pose_transform_point(&sets[i]->hand_pose.pose, &local, &out_corners[i]);
	}
}

// Compute the framed region (bounding box of the 4 fingertip corners projected
// into the left eye) and request a cropped screenshot. Falls back to full view
// if the head/projection isn't usable.
static void
request_framed_screenshot(struct u_gesture *g,
                          const struct xrt_hand_joint_set *left,
                          const struct xrt_hand_joint_set *right,
                          int64_t now_ns)
{
	struct xrt_device *head = g->xsysd->static_roles.head;
	if (head == NULL || head->get_view_poses == NULL) {
		u_screenshot_request();
		return;
	}

	struct xrt_vec3 eye_hint = {0.063f, 0.0f, 0.0f};
	struct xrt_space_relation head_rel;
	U_ZERO(&head_rel);
	struct xrt_fov fovs[2];
	struct xrt_pose eye_poses[2];
	xrt_result_t vr = xrt_device_get_view_poses(head, &eye_hint, now_ns, XRT_VIEW_TYPE_STEREO, 2, &head_rel,
	                                            fovs, eye_poses);
	if (vr != XRT_SUCCESS) {
		u_screenshot_request();
		return;
	}

	// Left-eye world pose -> view matrix. Head pose and hand poses are both in
	// the device tracking space, so the projection is self-consistent.
	struct xrt_pose eye_world;
	math_pose_transform(&head_rel.pose, &eye_poses[0], &eye_world);
	struct xrt_matrix_4x4 view;
	math_matrix_4x4_view_from_pose(&eye_world, &view);

	struct xrt_vec3 corners[4];
	compute_frame_world_corners(left, right, corners);

	float minu = 1e9f, minv = 1e9f, maxu = -1e9f, maxv = -1e9f;
	int ok = 0;
	for (int i = 0; i < 4; i++) {
		float u, v;
		if (!project_to_uv(&view, &fovs[0], corners[i], &u, &v)) {
			continue;
		}
		if (u < minu) {
			minu = u;
		}
		if (u > maxu) {
			maxu = u;
		}
		if (v < minv) {
			minv = v;
		}
		if (v > maxv) {
			maxv = v;
		}
		ok++;
	}

	if (ok == 4 && (maxu - minu) > 0.02f && (maxv - minv) > 0.02f) {
		U_LOG_I("gesture: frame rect u[%.2f..%.2f] v[%.2f..%.2f] -> cropped screenshot", minu, maxu, minv,
		        maxv);
		u_screenshot_request_rect(minu, minv, maxu, maxv);
	} else {
		U_LOG_I("gesture: projection unusable (ok=%d) -> full-view screenshot", ok);
		u_screenshot_request();
	}
}


/*
 *
 * Tick.
 *
 */

static void
detect_and_maybe_fire(struct u_gesture *g, int64_t now_ns)
{
	// Resolve the current left/right hands each tick (follows hotplug:
	// gloves hold these roles until real controllers power on, and back).
	struct xrt_system_roles roles = XRT_SYSTEM_ROLES_INIT;
	if (xrt_system_devices_get_roles(g->xsysd, &roles) != XRT_SUCCESS) {
		g->hold_start_ns = 0;
		return;
	}

	struct xrt_hand_joint_set left, right;
	bool have_left = get_hand(g->xsysd, roles.left, now_ns, &left);
	bool have_right = get_hand(g->xsysd, roles.right, now_ns, &right);

	bool detected = false;
	struct hand_pose lp = {0}, rp = {0};
	float wrist_dist = 0.0f;
	float idx_dot = 0.0f, thumb_dot = 0.0f;

	if (have_left && have_right) {
		lp = analyse_hand(&left);
		rp = analyse_hand(&right);

		// World-space hand separation (hand_pose is the world placement;
		// the per-joint positions are hand-local, so use hand_pose here).
		struct xrt_vec3 lw = left.hand_pose.pose.position;
		struct xrt_vec3 rw = right.hand_pose.pose.position;
		wrist_dist = v_len(v_sub(lw, rw));
		bool prox = wrist_dist > MIN_WRIST_DIST && wrist_dist < MAX_WRIST_DIST;

		// "Forms a frame": the two L-poses must oppose each other - index
		// fingers roughly anti-parallel and thumbs roughly anti-parallel.
		struct xrt_vec3 li = world_finger_dir(&left, XRT_HAND_JOINT_INDEX_PROXIMAL, XRT_HAND_JOINT_INDEX_TIP);
		struct xrt_vec3 ri = world_finger_dir(&right, XRT_HAND_JOINT_INDEX_PROXIMAL, XRT_HAND_JOINT_INDEX_TIP);
		struct xrt_vec3 lt = world_finger_dir(&left, XRT_HAND_JOINT_THUMB_PROXIMAL, XRT_HAND_JOINT_THUMB_TIP);
		struct xrt_vec3 rt = world_finger_dir(&right, XRT_HAND_JOINT_THUMB_PROXIMAL, XRT_HAND_JOINT_THUMB_TIP);
		idx_dot = v_dot(li, ri);
		thumb_dot = v_dot(lt, rt);
		bool opposed = idx_dot < FRAME_OPP_COS && thumb_dot < FRAME_OPP_COS;

		detected = lp.is_frame && rp.is_frame && prox && opposed;
	}

	// Throttled debug to help tune thresholds without a VR view.
	if (g->debug && now_ns - g->last_debug_ns > 500 * 1000 * 1000LL) {
		g->last_debug_ns = now_ns;
		if (have_left && have_right) {
			U_LOG_I("gesture: L[t%.2f i%.2f m%.2f r%.2f p%.2f f%d] "
			        "R[t%.2f i%.2f m%.2f r%.2f p%.2f f%d] dist=%.2f idot=%.2f tdot=%.2f -> %s",
			        lp.thumb, lp.index, lp.middle, lp.ring, lp.little, lp.is_frame, rp.thumb, rp.index,
			        rp.middle, rp.ring, rp.little, rp.is_frame, wrist_dist, idx_dot, thumb_dot,
			        detected ? "FRAME" : "-");
		} else {
			U_LOG_I("gesture: hands not both tracked (L=%d R=%d)", have_left, have_right);
		}
	}

	if (!detected) {
		g->hold_start_ns = 0;
		if (g->frame_feedback) {
			u_gesture_feedback_clear();
		}
		return;
	}

	if (g->hold_start_ns == 0) {
		g->hold_start_ns = now_ns;
	}

	// Publish the live in-headset rectangle (corners + hold progress) every
	// tick the pose is held.
	if (g->frame_feedback) {
		struct xrt_vec3 corners[4];
		compute_frame_world_corners(&left, &right, corners);
		float progress = (float)(now_ns - g->hold_start_ns) / (float)g->hold_ns;
		progress = progress < 0.0f ? 0.0f : (progress > 1.0f ? 1.0f : progress);
		u_gesture_feedback_publish(corners, progress);
	}

	if (now_ns - g->hold_start_ns >= g->hold_ns && now_ns - g->last_fire_ns >= g->cooldown_ns) {
		// Stop drawing before the shutter fires so the captured frame is
		// clean (no overlay lines burned into the screenshot).
		if (g->frame_feedback) {
			u_gesture_feedback_clear();
		}
		request_framed_screenshot(g, &left, &right, now_ns);
		g->last_fire_ns = now_ns;
		g->hold_start_ns = 0; // require releasing and re-forming the pose
	}
}

/*
 *
 * Config (XDG file + env overrides).
 *
 */

// Load settings: defaults, then ~/.config/monado/gestures.json, then env
// overrides (env wins, for developer convenience). Safe to call repeatedly.
static void
load_config(struct u_gesture *g)
{
	bool enabled = true;
	int hold_ms = DEFAULT_HOLD_MS;
	bool debug = false;
	bool frame_feedback = true;

	if (g->config_path[0] != '\0') {
		size_t size = 0;
		char *content = u_file_read_content_from_path(g->config_path, &size);
		if (content != NULL) {
			cJSON *root = cJSON_Parse(content);
			if (root != NULL) {
				u_json_get_bool(u_json_get(root, "enabled"), &enabled);
				u_json_get_int(u_json_get(root, "hold_ms"), &hold_ms);
				u_json_get_bool(u_json_get(root, "debug"), &debug);
				u_json_get_bool(u_json_get(root, "frame_feedback"), &frame_feedback);
				cJSON_Delete(root);
			}
			free(content);
		}
	}

	if (getenv("MONADO_GESTURE_DISABLE") != NULL) {
		enabled = false;
	}
	const char *he = getenv("MONADO_GESTURE_HOLD_MS");
	if (he != NULL) {
		long v = strtol(he, NULL, 10);
		if (v > 0) {
			hold_ms = (int)v;
		}
	}
	if (getenv("MONADO_GESTURE_DEBUG") != NULL) {
		debug = true;
	}
	if (getenv("MONADO_GESTURE_NO_FEEDBACK") != NULL) {
		frame_feedback = false;
	}

	if (hold_ms < 100) {
		hold_ms = 100; // sanity floor
	}

	g->enabled = enabled;
	g->hold_ns = (int64_t)hold_ms * 1000 * 1000LL;
	g->debug = debug;
	g->frame_feedback = frame_feedback;
}

// Reload the config file when it changes on disk (so the control app can toggle
// the gesture / change the delay live). Polled at ~1 Hz.
static void
maybe_reload_config(struct u_gesture *g, int64_t now_ns)
{
#if !defined(_WIN32)
	if (g->config_path[0] == '\0' || now_ns - g->last_config_check_ns < 1000 * 1000 * 1000LL) {
		return;
	}
	g->last_config_check_ns = now_ns;

	struct stat st;
	if (stat(g->config_path, &st) != 0) {
		return; // no file yet; keep current settings
	}
	if ((int64_t)st.st_mtime == g->config_mtime) {
		return;
	}
	g->config_mtime = (int64_t)st.st_mtime;
	load_config(g);
	U_LOG_I("gesture: reloaded config (enabled=%d hold=%lldms debug=%d frame_feedback=%d)", g->enabled,
	        (long long)(g->hold_ns / (1000 * 1000)), g->debug, g->frame_feedback);

	// Drop any lingering overlay if the feature was just turned off.
	if (!g->enabled || !g->frame_feedback) {
		u_gesture_feedback_clear();
	}
#else
	(void)g;
	(void)now_ns;
#endif
}


static void *
gesture_run(void *ptr)
{
	struct u_gesture *g = (struct u_gesture *)ptr;

	while (os_thread_helper_is_running(&g->oth)) {
		int64_t now_ns = os_monotonic_get_ns();
		maybe_reload_config(g, now_ns);
		if (g->enabled) {
			detect_and_maybe_fire(g, now_ns);
		}
		os_nanosleep(TICK_NS);
	}

	return NULL;
}


/*
 *
 * 'Exported' functions.
 *
 */

int
u_gesture_create(struct xrt_system_devices *xsysd, struct u_gesture **out_gesture)
{
	if (xsysd == NULL) {
		return -1;
	}

	struct u_gesture *g = U_TYPED_CALLOC(struct u_gesture);
	g->xsysd = xsysd;
	g->cooldown_ns = COOLDOWN_MS * 1000 * 1000LL;

	// Config file: <XDG config>/monado/gestures.json (written by the control app).
	if (u_file_get_path_in_config_dir("gestures.json", g->config_path, sizeof(g->config_path)) <= 0) {
		g->config_path[0] = '\0';
	}
#if !defined(_WIN32)
	struct stat st;
	if (g->config_path[0] != '\0' && stat(g->config_path, &st) == 0) {
		g->config_mtime = (int64_t)st.st_mtime;
	}
#endif
	load_config(g); // defaults + file + env overrides

	int ret = os_thread_helper_init(&g->oth);
	if (ret != 0) {
		U_LOG_E("gesture: os_thread_helper_init failed");
		free(g);
		return ret;
	}

	ret = os_thread_helper_start(&g->oth, gesture_run, g);
	if (ret != 0) {
		U_LOG_E("gesture: os_thread_helper_start failed");
		os_thread_helper_destroy(&g->oth);
		free(g);
		return ret;
	}

	U_LOG_I("gesture: detector started (enabled=%d hold=%lldms, config=%s)", g->enabled,
	        (long long)(g->hold_ns / (1000 * 1000)), g->config_path[0] != '\0' ? g->config_path : "(none)");

	*out_gesture = g;
	return 0;
}

void
u_gesture_destroy(struct u_gesture **ptr_gesture)
{
	struct u_gesture *g = *ptr_gesture;
	if (g == NULL) {
		return;
	}

	os_thread_helper_destroy(&g->oth);
	free(g);
	*ptr_gesture = NULL;
}
