// Copyright 2018-2024, Collabora, Ltd.
// Copyright 2023-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Helper functions for device role getting.
 * @ingroup oxr_main
 */

#pragma once

#include "oxr_objects.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 *
 * Static device roles.
 *
 */

// clang-format off
static inline struct xrt_device *get_static_role_head(struct oxr_system *sys) { return sys->xsysd->static_roles.head; }
static inline struct xrt_device *get_static_role_eyes(struct oxr_system *sys) { return sys->xsysd->static_roles.eyes; }
static inline struct xrt_device *get_static_role_face(struct oxr_system* sys) { return sys->xsysd->static_roles.face; }
static inline struct xrt_device *get_static_role_body(struct oxr_system* sys) { return sys->xsysd->static_roles.body; }
static inline struct xrt_device *get_static_role_hand_tracking_unobstructed_left(struct oxr_system* sys) { return sys->xsysd->static_roles.hand_tracking.unobstructed.left; }
static inline struct xrt_device *get_static_role_hand_tracking_unobstructed_right(struct oxr_system* sys) { return sys->xsysd->static_roles.hand_tracking.unobstructed.right; }
static inline struct xrt_device *get_static_role_hand_tracking_conforming_left(struct oxr_system* sys) { return sys->xsysd->static_roles.hand_tracking.conforming.left; }
static inline struct xrt_device *get_static_role_hand_tracking_conforming_right(struct oxr_system* sys) { return sys->xsysd->static_roles.hand_tracking.conforming.right; }
// clang-format on

#define GET_STATIC_XDEV_BY_ROLE(SYS, ROLE) (get_static_role_##ROLE((SYS)))


/*
 *
 * Dynamic device roles.
 *
 */

// clang-format off
#define STATIC_WRAP(ROLE)                                                                                              \
	static inline struct xrt_device *get_role_##ROLE(struct oxr_session *sess)                                     \
	{                                                                                                              \
		return get_static_role_##ROLE(sess->sys);                                                              \
	}
STATIC_WRAP(head)
STATIC_WRAP(eyes)
STATIC_WRAP(face)
STATIC_WRAP(body)
STATIC_WRAP(hand_tracking_unobstructed_left)
STATIC_WRAP(hand_tracking_unobstructed_right)
STATIC_WRAP(hand_tracking_conforming_left)
STATIC_WRAP(hand_tracking_conforming_right)
#undef STATIC_WRAP
// clang-format on

#define MAKE_GET_DYN_ROLES_FN(ROLE)                                                                                    \
	static inline struct xrt_device *get_role_##ROLE(struct oxr_session *sess)                                     \
	{                                                                                                              \
		const bool is_locked = 0 == os_mutex_trylock(&sess->sys->sync_actions_mutex);                          \
		const int32_t xdev_idx = sess->sys->dynamic_roles_cache.ROLE;                                          \
		if (is_locked) {                                                                                       \
			os_mutex_unlock(&sess->sys->sync_actions_mutex);                                               \
		}                                                                                                      \
		struct xrt_system_devices *xsysd = sess->sys->xsysd;                                                   \
		if (xdev_idx < 0 || xdev_idx >= (int32_t)xsysd->xdev_count) {                                          \
			return NULL;                                                                                   \
		}                                                                                                      \
		return xsysd->xdevs[xdev_idx];                                                                         \
	}
MAKE_GET_DYN_ROLES_FN(left)
MAKE_GET_DYN_ROLES_FN(right)
MAKE_GET_DYN_ROLES_FN(gamepad)
#undef MAKE_GET_DYN_ROLES_FN

#define GET_XDEV_BY_ROLE(SESS, ROLE) (get_role_##ROLE((SESS)))


/*
 *
 * Dynamic profile roles.
 *
 */

static inline enum xrt_device_name
get_role_profile_head(struct oxr_session *sess)
{
	return XRT_DEVICE_INVALID;
}
static inline enum xrt_device_name
get_role_profile_eyes(struct oxr_session *sess)
{
	return XRT_DEVICE_INVALID;
}
static inline enum xrt_device_name
get_role_profile_face(struct oxr_session *sess)
{
	return XRT_DEVICE_INVALID;
}
static inline enum xrt_device_name
get_role_profile_body(struct oxr_session *sess)
{
	return XRT_DEVICE_INVALID;
}
static inline enum xrt_device_name
get_role_profile_hand_tracking_unobstructed_left(struct oxr_session *sess)
{
	return XRT_DEVICE_INVALID;
}
static inline enum xrt_device_name
get_role_profile_hand_tracking_unobstructed_right(struct oxr_session *sess)
{
	return XRT_DEVICE_INVALID;
}

static inline enum xrt_device_name
get_role_profile_hand_tracking_conforming_left(struct oxr_session *sess)
{
	return XRT_DEVICE_INVALID;
}
static inline enum xrt_device_name
get_role_profile_hand_tracking_conforming_right(struct oxr_session *sess)
{
	return XRT_DEVICE_INVALID;
}

#define MAKE_GET_DYN_ROLE_PROFILE_FN(ROLE)                                                                             \
	static inline enum xrt_device_name get_role_profile_##ROLE(struct oxr_session *sess)                           \
	{                                                                                                              \
		const bool is_locked = 0 == os_mutex_trylock(&sess->sys->sync_actions_mutex);                          \
		const enum xrt_device_name profile_name = sess->sys->dynamic_roles_cache.ROLE##_profile;               \
		if (is_locked) {                                                                                       \
			os_mutex_unlock(&sess->sys->sync_actions_mutex);                                               \
		}                                                                                                      \
		return profile_name;                                                                                   \
	}
MAKE_GET_DYN_ROLE_PROFILE_FN(left)
MAKE_GET_DYN_ROLE_PROFILE_FN(right)
MAKE_GET_DYN_ROLE_PROFILE_FN(gamepad)
#undef MAKE_GET_DYN_ROLES_FN

#define GET_PROFILE_NAME_BY_ROLE(SYS, ROLE) (get_role_profile_##ROLE((SYS)))


#ifdef __cplusplus
}
#endif
