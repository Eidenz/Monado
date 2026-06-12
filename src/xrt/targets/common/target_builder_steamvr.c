// Copyright 2023, Duncan Spaulding.
// Copyright 2022-2023, Collabora, Ltd.
// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Builder for SteamVR proprietary driver wrapper.
 * @author BabbleBones <BabbleBones@protonmail.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup xrt_iface
 */
#include "xrt/xrt_config_drivers.h"

#include <assert.h>
#include <stdbool.h>

#include "tracking/t_tracking.h"

#include "os/os_threading.h"

#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"

#include "util/u_debug.h"
#include "util/u_system_helpers.h"

#include "vive/vive_builder.h"

#include "target_builder_interface.h"

#include "steamvr_lh/steamvr_lh_interface.h"
#include "xrt/xrt_results.h"

#ifdef XRT_BUILD_DRIVER_UDCAP
#include "udcap/udcap_interface.h"
#endif

#include "xrt/xrt_space.h"
#include "b_space_overseer.h"

#ifndef XRT_BUILD_DRIVER_STEAMVR_LIGHTHOUSE
#error "This builder requires the SteamVR Lighthouse driver"
#endif

DEBUG_GET_ONCE_LOG_OPTION(svr_log, "STEAMVR_LH_LOG", U_LOGGING_INFO)

#define SVR_TRACE(...) U_LOG_IFL_T(debug_get_log_option_svr_log(), __VA_ARGS__)
#define SVR_DEBUG(...) U_LOG_IFL_D(debug_get_log_option_svr_log(), __VA_ARGS__)
#define SVR_INFO(...) U_LOG_IFL_I(debug_get_log_option_svr_log(), __VA_ARGS__)
#define SVR_WARN(...) U_LOG_IFL_W(debug_get_log_option_svr_log(), __VA_ARGS__)
#define SVR_ERROR(...) U_LOG_IFL_E(debug_get_log_option_svr_log(), __VA_ARGS__)
#define SVR_ASSERT(predicate, ...)                                                                                     \
	do {                                                                                                           \
		bool p = predicate;                                                                                    \
		if (!p) {                                                                                              \
			U_LOG(U_LOGGING_ERROR, __VA_ARGS__);                                                           \
			assert(false && "SVR_ASSERT failed: " #predicate);                                             \
			exit(EXIT_FAILURE);                                                                            \
		}                                                                                                      \
	} while (false);
#define SVR_ASSERT_(predicate) SVR_ASSERT(predicate, "Assertion failed " #predicate)


/*
 *
 * Misc stuff.
 *
 */

DEBUG_GET_ONCE_BOOL_OPTION(steamvr_enable, "STEAMVR_LH_ENABLE", false)

static const char *driver_list[] = {
    "steamvr_lh",
};

struct steamvr_builder
{
	struct xrt_builder base;

	struct xrt_device *head;

	struct
	{
		struct
		{
			struct xrt_device *left, *right;
		} unobstructed;

		struct
		{
			struct xrt_device *left, *right;
		} conforming;
	} hand_tracking;

	bool is_valve_index;
};

/*
 *
 * Member functions.
 *
 */

static xrt_result_t
steamvr_estimate_system(struct xrt_builder *xb,
                        cJSON *config,
                        struct xrt_prober *xp,
                        struct xrt_builder_estimate *estimate)
{
	struct steamvr_builder *svrb = (struct steamvr_builder *)xb;

	// Currently no built in support for hand tracking.
	bool have_hand_tracking = false;

	if (debug_get_bool_option_steamvr_enable()) {
		return vive_builder_estimate( //
		    xp,                       // xp
		    true,                     // have_6dof
		    have_hand_tracking,       // have_hand_tracking
		    &svrb->is_valve_index,    // out_have_valve_index
		    estimate);                // out_estimate
	} else {
		return XRT_SUCCESS;
	}
}

static void
steamvr_destroy(struct xrt_builder *xb)
{
	struct steamvr_builder *svrb = (struct steamvr_builder *)xb;
	free(svrb);
}

#ifdef XRT_BUILD_DRIVER_UDCAP
// steamvr_lh installs its own get_roles, which only assigns the left/right
// controller roles from its own connected controllers (it ignores our foreign
// glove devices). We wrap it so any hand it leaves unassigned falls back to
// the matching glove: with the controllers off the gloves hold the roles, and
// powering controllers on (or off) mid-session hands the roles over.
//
// The wrapper keeps its own cached roles + generation since the final
// assignment differs from the inner driver's, and callers (the IPC handler)
// pass in a fresh zeroed struct every call.
static xrt_result_t (*udcap_orig_get_roles)(struct xrt_system_devices *, struct xrt_system_roles *) = NULL;
static struct xrt_device *udcap_dev_left = NULL;
static struct xrt_device *udcap_dev_right = NULL;
static struct os_mutex udcap_roles_mutex;
static struct xrt_system_roles udcap_cached_roles = XRT_SYSTEM_ROLES_INIT;

static xrt_result_t
udcap_get_roles(struct xrt_system_devices *xsysd, struct xrt_system_roles *out_roles)
{
	struct xrt_system_roles inner = XRT_SYSTEM_ROLES_INIT;
	xrt_result_t xret = udcap_orig_get_roles(xsysd, &inner);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	// Find the glove devices.
	int32_t li = -1, ri = -1;
	for (size_t i = 0; i < xsysd->static_xdev_count; i++) {
		if (udcap_dev_left != NULL && xsysd->static_xdevs[i] == udcap_dev_left) {
			li = (int32_t)i;
		}
		if (udcap_dev_right != NULL && xsysd->static_xdevs[i] == udcap_dev_right) {
			ri = (int32_t)i;
		}
	}

	/*
	 * A hand goes to the glove unless a *connected* controller holds it.
	 * The driver keeps roles sticky on powered-off controllers (so hands
	 * freeze in place when there are no gloves), but with gloves present
	 * a live glove beats a powered-off controller — this is what makes
	 * controllers-off hand the roles back to the gloves mid-session.
	 * A dead glove (udcap-server gone) never takes a role, so killing the
	 * server hands the roles to the controllers (or freezes them).
	 */
	int32_t left = inner.left;
	int32_t right = inner.right;
	enum xrt_device_name left_profile = inner.left_profile;
	enum xrt_device_name right_profile = inner.right_profile;

	bool left_live = left >= 0 && (size_t)left < xsysd->static_xdev_count &&
	                 steamvr_lh_device_is_connected(xsysd->static_xdevs[left]);
	bool right_live = right >= 0 && (size_t)right < xsysd->static_xdev_count &&
	                  steamvr_lh_device_is_connected(xsysd->static_xdevs[right]);

	if (li >= 0 && !left_live && udcap_device_is_alive(udcap_dev_left)) {
		left = li;
		left_profile = XRT_DEVICE_INDEX_CONTROLLER;
	}
	if (ri >= 0 && !right_live && udcap_device_is_alive(udcap_dev_right)) {
		right = ri;
		right_profile = XRT_DEVICE_INDEX_CONTROLLER;
	}

	os_mutex_lock(&udcap_roles_mutex);

	if (left != udcap_cached_roles.left || right != udcap_cached_roles.right ||
	    inner.gamepad != udcap_cached_roles.gamepad) {
		SVR_INFO("udcap roles changed: left=%d (%s) right=%d (%s)", //
		         left, left >= 0 ? xsysd->static_xdevs[left]->str : "<none>", right,
		         right >= 0 ? xsysd->static_xdevs[right]->str : "<none>");

		udcap_cached_roles.generation_id++;
		udcap_cached_roles.left = left;
		udcap_cached_roles.right = right;
		udcap_cached_roles.gamepad = inner.gamepad;
		udcap_cached_roles.left_profile = left_profile;
		udcap_cached_roles.right_profile = right_profile;
		udcap_cached_roles.gamepad_profile = inner.gamepad_profile;
	}

	*out_roles = udcap_cached_roles;

	os_mutex_unlock(&udcap_roles_mutex);

	return XRT_SUCCESS;
}
#endif

// Create UDCAP glove devices (if udcap-server is running), attach each to a
// SteamVR-tracked tracker, set them as the hand-tracking sources, and take over
// controller-role assignment so they become the left/right controllers.
static void
try_add_udcap(struct xrt_system_devices *xsysd)
{
#ifdef XRT_BUILD_DRIVER_UDCAP
	struct xrt_device *ud_left = NULL, *ud_right = NULL;
	udcap_create_devices(xsysd->static_xdevs, xsysd->static_xdev_count, &ud_left, &ud_right);
	if (ud_left == NULL && ud_right == NULL) {
		return;
	}

	if (ud_left != NULL) {
		if (xsysd->static_xdev_count < XRT_SYSTEM_MAX_DEVICES) {
			xsysd->static_xdevs[xsysd->static_xdev_count++] = ud_left;
		}
		xsysd->static_roles.hand_tracking.unobstructed.left = ud_left;
	}
	if (ud_right != NULL) {
		if (xsysd->static_xdev_count < XRT_SYSTEM_MAX_DEVICES) {
			xsysd->static_xdevs[xsysd->static_xdev_count++] = ud_right;
		}
		xsysd->static_roles.hand_tracking.unobstructed.right = ud_right;
	}

	udcap_dev_left = ud_left;
	udcap_dev_right = ud_right;
	udcap_orig_get_roles = xsysd->get_roles;
	os_mutex_init(&udcap_roles_mutex);
	udcap_cached_roles.generation_id = 1; // Valid generations start at 1.
	xsysd->get_roles = udcap_get_roles;
#else
	(void)xsysd;
#endif
}

// For filling hand-tracking roles from the device-added callback.
static struct xrt_system_devices *steamvr_xsysd = NULL;

/*
 * The static hand-tracking roles are normally fixed at boot. When a device
 * with skeletal tracking (e.g. Index controllers) is hotplugged into an empty
 * conforming slot, fill it in so hand trackers (which re-resolve sources
 * lazily) can pick it up for finger tracking.
 */
static void
steamvr_fill_ht_roles(struct xrt_device *xdev)
{
	if (steamvr_xsysd == NULL) {
		return;
	}

	for (uint32_t i = 0; i < xdev->input_count; i++) {
		const struct xrt_input *input = &xdev->inputs[i];
		if (!input->active) {
			continue;
		}

		if (input->name == XRT_INPUT_HT_CONFORMING_LEFT &&
		    steamvr_xsysd->static_roles.hand_tracking.conforming.left == NULL) {
			SVR_INFO("Hotplugged '%s' fills the conforming.left hand-tracking role", xdev->str);
			steamvr_xsysd->static_roles.hand_tracking.conforming.left = xdev;
		}
		if (input->name == XRT_INPUT_HT_CONFORMING_RIGHT &&
		    steamvr_xsysd->static_roles.hand_tracking.conforming.right == NULL) {
			SVR_INFO("Hotplugged '%s' fills the conforming.right hand-tracking role", xdev->str);
			steamvr_xsysd->static_roles.hand_tracking.conforming.right = xdev;
		}
	}
}

// Hotplugged devices need a space before clients can locate poses on them.
static void
steamvr_on_device_added(struct xrt_device *xdev, void *userdata)
{
	struct xrt_space_overseer *xso = (struct xrt_space_overseer *)userdata;

	xrt_result_t xret = xrt_space_overseer_add_device(xso, xdev);
	if (xret != XRT_SUCCESS) {
		SVR_ERROR("Failed to add hotplugged device '%s' to the space overseer", xdev->str);
	}

	steamvr_fill_ht_roles(xdev);

#ifdef XRT_BUILD_DRIVER_UDCAP
	// A glove whose tracker wasn't on at boot can pick it up now.
	udcap_notify_device_added(xdev);
#endif
}

static xrt_result_t
steamvr_open_system(struct xrt_builder *xb,
                    cJSON *config,
                    struct xrt_prober *xp,
                    struct xrt_session_event_sink *broadcast,
                    struct xrt_system_devices **out_xsysd,
                    struct xrt_space_overseer **out_xso)
{
	struct steamvr_builder *svrb = (struct steamvr_builder *)xb;

	assert(out_xsysd != NULL);
	assert(*out_xsysd == NULL);

	enum xrt_result result = steamvr_lh_create_devices(xp, out_xsysd);

	if (result != XRT_SUCCESS) {
		SVR_ERROR("Unable to create devices");
		return result;
	}

	struct xrt_system_devices *xsysd = NULL;
	xsysd = *out_xsysd;

	if (xsysd->static_roles.head == NULL) {
		SVR_ERROR("Unable to find HMD");
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	svrb->head = xsysd->static_roles.head;

#define SET_HT_ROLES(SRC)                                                                                              \
	svrb->hand_tracking.SRC.left = u_system_devices_get_ht_device_##SRC##_left(xsysd);                             \
	svrb->hand_tracking.SRC.right = u_system_devices_get_ht_device_##SRC##_right(xsysd);                           \
	xsysd->static_roles.hand_tracking.SRC.left = svrb->hand_tracking.SRC.left;                                     \
	xsysd->static_roles.hand_tracking.SRC.right = svrb->hand_tracking.SRC.right;
	SET_HT_ROLES(unobstructed)
	SET_HT_ROLES(conforming)
#undef SET_HT_ROLES

	// UDCAP gloves: override hand-tracking roles + attach to trackers.
	try_add_udcap(xsysd);

	/*
	 * Space overseer.
	 */

	struct b_space_overseer *uso = b_space_overseer_create(broadcast);

	struct xrt_pose T_stage_local = XRT_POSE_IDENTITY;

	b_space_overseer_legacy_setup( //
	    uso,                       // uso
	    xsysd->static_xdevs,       // xdevs
	    xsysd->static_xdev_count,  // xdev_count
	    svrb->head,                // head
	    &T_stage_local,            // local_offset
	    false,                     // root_is_unbounded
	    true                       // per_app_local_spaces
	);

	*out_xso = (struct xrt_space_overseer *)uso;

	// Wire up hotplug: devices appearing from now on get a space added.
	steamvr_xsysd = xsysd;
	steamvr_lh_set_device_added_callback(xsysd, steamvr_on_device_added, *out_xso);

	return result;
}


/*
 *
 * 'Exported' functions.
 *
 */

struct xrt_builder *
t_builder_steamvr_create(void)
{
	struct steamvr_builder *svrb = U_TYPED_CALLOC(struct steamvr_builder);
	svrb->base.estimate_system = steamvr_estimate_system;
	svrb->base.open_system = steamvr_open_system;
	svrb->base.destroy = steamvr_destroy;
	svrb->base.identifier = "steamvr";
	svrb->base.name = "SteamVR proprietary wrapper (Vive, Index, Tundra trackers, etc.) devices builder";
	svrb->base.driver_identifiers = driver_list;
	svrb->base.driver_identifier_count = ARRAY_SIZE(driver_list);

	return &svrb->base;
}
