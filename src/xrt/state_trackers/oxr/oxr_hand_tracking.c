// Copyright 2019-2024, Collabora, Ltd.
// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Hand tracking functions.
 * @author Christoph Haag <christoph.haag@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup oxr_main
 */

#include "xrt/xrt_compiler.h"

#include "math/m_api.h"
#include "math/m_mathinclude.h"
#include "math/m_space.h"

#include "util/u_debug.h"
#include "util/u_trace_marker.h"

#include "oxr_hand_tracking.h"
#include "oxr_conversions.h"
#include "oxr_logger.h"
#include "oxr_chain.h"
#include "oxr_xret.h"
#include "oxr_roles.h"


/*
 *
 * Helpers
 *
 */

DEBUG_GET_ONCE_BOOL_OPTION(hand_tracking_prioritize_conforming, "OXR_HAND_TRACKING_PRIORITIZE_CONFORMING", false)

static void
xrt_to_xr_pose(struct xrt_pose *xrt_pose, XrPosef *xr_pose)
{
	xr_pose->orientation.x = xrt_pose->orientation.x;
	xr_pose->orientation.y = xrt_pose->orientation.y;
	xr_pose->orientation.z = xrt_pose->orientation.z;
	xr_pose->orientation.w = xrt_pose->orientation.w;

	xr_pose->position.x = xrt_pose->position.x;
	xr_pose->position.y = xrt_pose->position.y;
	xr_pose->position.z = xrt_pose->position.z;
}

static enum xrt_output_name
xr_hand_to_force_feedback_output(XrHandEXT hand)
{
	switch (hand) {
	case XR_HAND_LEFT_EXT: return XRT_OUTPUT_NAME_FORCE_FEEDBACK_LEFT;
	case XR_HAND_RIGHT_EXT: return XRT_OUTPUT_NAME_FORCE_FEEDBACK_RIGHT;
	default: assert(false); return 0;
	}
}


/*
 *
 * XR_EXT_hand_tracking
 *
 */

#ifdef OXR_HAVE_EXT_hand_tracking

static XrResult
oxr_hand_tracker_destroy_cb(struct oxr_logger *log, struct oxr_handle_base *hb)
{
	struct oxr_hand_tracker *hand_tracker = (struct oxr_hand_tracker *)hb;

	free(hand_tracker);

	return XR_SUCCESS;
}

XrResult
oxr_hand_tracker_create(struct oxr_logger *log,
                        struct oxr_session *sess,
                        const XrHandTrackerCreateInfoEXT *createInfo,
                        struct oxr_hand_tracker **out_hand_tracker)
{
	if (!oxr_system_get_hand_tracking_support(log, sess->sys->inst)) {
		return oxr_error(log, XR_ERROR_FEATURE_UNSUPPORTED, "System does not support hand tracking");
	}

	struct oxr_hand_tracker *hand_tracker = NULL;
	OXR_ALLOCATE_HANDLE_OR_RETURN(log, hand_tracker, OXR_XR_DEBUG_HTRACKER, oxr_hand_tracker_destroy_cb,
	                              &sess->handle);

	hand_tracker->sess = sess;
	hand_tracker->hand = createInfo->hand;
	hand_tracker->hand_joint_set = createInfo->handJointSet;

#define OXR_SET_HT_DATA_SOURCE(SRC, SRC_TYPE)                                                                          \
	{                                                                                                              \
		struct xrt_device *xdev = NULL;                                                                        \
		if (createInfo->hand == XR_HAND_LEFT_EXT) {                                                            \
			xdev = GET_STATIC_XDEV_BY_ROLE(sess->sys, hand_tracking_##SRC##_left);                         \
		} else if (createInfo->hand == XR_HAND_RIGHT_EXT) {                                                    \
			xdev = GET_STATIC_XDEV_BY_ROLE(sess->sys, hand_tracking_##SRC##_right);                        \
		}                                                                                                      \
                                                                                                                       \
		if (xdev != NULL && xdev->supported.hand_tracking) {                                                   \
			const enum xrt_input_name ht_input_name = createInfo->hand == XR_HAND_LEFT_EXT                 \
			                                              ? XRT_INPUT_HT_##SRC_TYPE##_LEFT                 \
			                                              : XRT_INPUT_HT_##SRC_TYPE##_RIGHT;               \
			struct xrt_input *input = NULL;                                                                \
			if (oxr_xdev_find_input(xdev, ht_input_name, &input) && input != NULL) {                       \
				hand_tracker->SRC = (struct oxr_hand_tracking_data_source){                            \
				    .xdev = xdev,                                                                      \
				    .input_name = ht_input_name,                                                       \
				};                                                                                     \
			}                                                                                              \
		}                                                                                                      \
                                                                                                                       \
		if (xdev != NULL && hand_tracker->SRC.xdev == NULL)                                                    \
			oxr_warn(log, "We got hand tracking xdev (%s) but it didn't have a hand tracking input.",      \
			         #SRC);                                                                                \
	}

	// Find the assigned device.
	OXR_SET_HT_DATA_SOURCE(unobstructed, UNOBSTRUCTED)
	OXR_SET_HT_DATA_SOURCE(conforming, CONFORMING)
#undef OXR_SET_HT_DATA_SOURCE

	hand_tracker->requested_sources_count = ARRAY_SIZE(hand_tracker->requested_sources);
	hand_tracker->requested_sources[0] = &hand_tracker->unobstructed;
	hand_tracker->requested_sources[1] = &hand_tracker->conforming;

#ifdef OXR_HAVE_EXT_hand_tracking_data_source
	const XrHandTrackingDataSourceInfoEXT *data_source_info = NULL;
	if (sess->sys->inst->extensions.EXT_hand_tracking_data_source) {
		data_source_info = OXR_GET_INPUT_FROM_CHAIN(createInfo, XR_TYPE_HAND_TRACKING_DATA_SOURCE_INFO_EXT,
		                                            XrHandTrackingDataSourceInfoEXT);
	}

	if (data_source_info != NULL) {

		const uint32_t source_count =
		    MIN(data_source_info->requestedDataSourceCount, hand_tracker->requested_sources_count);
		hand_tracker->requested_sources_count = 0;
		memset(hand_tracker->requested_sources, 0, sizeof(hand_tracker->requested_sources));

		for (uint32_t i = 0; i < source_count; ++i) {
			struct oxr_hand_tracking_data_source *requested_source = NULL;
			switch (data_source_info->requestedDataSources[i]) {
			case XR_HAND_TRACKING_DATA_SOURCE_UNOBSTRUCTED_EXT:
				requested_source = &hand_tracker->unobstructed;
				break;
			case XR_HAND_TRACKING_DATA_SOURCE_CONTROLLER_EXT:
				requested_source = &hand_tracker->conforming;
				break;
			default: break;
			}
			if (requested_source && requested_source->xdev != NULL) {
				hand_tracker->requested_sources[hand_tracker->requested_sources_count++] =
				    requested_source;
			}
		}

		if (hand_tracker->requested_sources_count == 0) {
			return oxr_error(
			    log, XR_ERROR_FEATURE_UNSUPPORTED,
			    "None of the requested data sources are supported by the current hand-tracking device(s).");
		}

		const size_t sort_size = hand_tracker->requested_sources_count;
		const size_t elem_size = sizeof(const struct oxr_hand_tracking_data_source *);
		qsort(hand_tracker->requested_sources, sort_size, elem_size, oxr_hand_tracking_data_source_cmp);
	}
#endif

	*out_hand_tracker = hand_tracker;

	return XR_SUCCESS;
}

XrResult
oxr_hand_tracker_joints(struct oxr_logger *log,
                        struct oxr_hand_tracker *hand_tracker,
                        const XrHandJointsLocateInfoEXT *locateInfo,
                        XrHandJointLocationsEXT *locations)
{
	XrResult ret = XR_SUCCESS;

	struct oxr_space *baseSpc = XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_space *, locateInfo->baseSpace);

	struct oxr_session *sess = hand_tracker->sess;
	struct oxr_instance *inst = sess->sys->inst;

	XrHandJointVelocitiesEXT *vel =
	    OXR_GET_OUTPUT_FROM_CHAIN(locations, XR_TYPE_HAND_JOINT_VELOCITIES_EXT, XrHandJointVelocitiesEXT);

#ifdef OXR_HAVE_EXT_hand_tracking_data_source
	XrHandTrackingDataSourceStateEXT *data_source_state = NULL;
	if (hand_tracker->sess->sys->inst->extensions.EXT_hand_tracking_data_source) {
		data_source_state = OXR_GET_OUTPUT_FROM_CHAIN(locations, XR_TYPE_HAND_TRACKING_DATA_SOURCE_STATE_EXT,
		                                              XrHandTrackingDataSourceStateEXT);
	}
#endif

	const XrTime at_time = locateInfo->time;

	//! Convert at_time to monotonic and give to device.
	const int64_t at_timestamp_ns = time_state_ts_to_monotonic_ns(inst->timekeeping, at_time);

	struct oxr_hand_tracking_data_source *data_sources[ARRAY_SIZE(hand_tracker->requested_sources)] = {0};
	memcpy(data_sources, hand_tracker->requested_sources, sizeof(data_sources));

	if (debug_get_bool_option_hand_tracking_prioritize_conforming() && //
	    hand_tracker->requested_sources_count > 1) {
		struct oxr_hand_tracking_data_source *tmp = data_sources[0];
		data_sources[0] = data_sources[1];
		data_sources[1] = tmp;
	}

	struct xrt_hand_joint_set value;
	const struct oxr_hand_tracking_data_source *data_source = NULL;
	for (uint32_t i = 0; i < hand_tracker->requested_sources_count; ++i) {
		data_source = data_sources[i];
		if (data_source == NULL || data_source->xdev == NULL)
			continue;
		int64_t ignored;
		value = (struct xrt_hand_joint_set){0};
		xrt_result_t xret = xrt_device_get_hand_tracking(data_source->xdev, data_source->input_name,
		                                                 at_timestamp_ns, &value, &ignored);
		OXR_CHECK_XRET_GOTO(log, sess, xret, xrt_device_get_hand_tracking, ret, hand_tracking_inactive);
		if (value.is_active) {
			break;
		}
	}

	if (data_source == NULL || data_source->xdev == NULL) {
		goto hand_tracking_inactive;
	}

#ifdef OXR_HAVE_EXT_hand_tracking_data_source
	if (data_source_state != NULL) {
		data_source_state->isActive = XR_TRUE;
		data_source_state->dataSource = xrt_hand_tracking_data_source_to_xr(data_source->input_name);
	}
#endif

	// The hand pose is returned in the xdev's space.
	struct xrt_space_relation T_xdev_hand = value.hand_pose;

	// Get the xdev's pose in the base space.
	struct xrt_space_relation T_base_xdev = XRT_SPACE_RELATION_ZERO;

	ret = oxr_space_locate_device(log, data_source->xdev, baseSpc, at_time, &T_base_xdev);
	if (ret != XR_SUCCESS || T_base_xdev.relation_flags == 0) {
		// Error printed logged oxr_space_locate_device
		goto hand_tracking_inactive;
	}

	// Get the hands pose in the base space.
	struct xrt_space_relation T_base_hand;
	struct xrt_relation_chain xrc = {0};
	m_relation_chain_push_relation(&xrc, &T_xdev_hand);
	m_relation_chain_push_relation(&xrc, &T_base_xdev);
	m_relation_chain_resolve(&xrc, &T_base_hand);

	// Can we not relate to this space or did we not get values?
	if (T_base_hand.relation_flags == 0 || !value.is_active) {
		goto hand_tracking_inactive;
	}

	// We know we are active.
	locations->isActive = true;

	for (uint32_t i = 0; i < locations->jointCount; i++) {
		locations->jointLocations[i].locationFlags =
		    xrt_to_xr_space_location_flags(value.values.hand_joint_set_default[i].relation.relation_flags);
		locations->jointLocations[i].radius = value.values.hand_joint_set_default[i].radius;

		struct xrt_space_relation r = value.values.hand_joint_set_default[i].relation;

		struct xrt_space_relation result;
		struct xrt_relation_chain chain = {0};
		m_relation_chain_push_relation(&chain, &r);
		m_relation_chain_push_relation(&chain, &T_base_hand);
		m_relation_chain_resolve(&chain, &result);

		xrt_to_xr_pose(&result.pose, &locations->jointLocations[i].pose);

		if (vel) {
			XrHandJointVelocityEXT *v = &vel->jointVelocities[i];

			v->velocityFlags = 0;
			if ((result.relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT)) {
				v->velocityFlags |= XR_SPACE_VELOCITY_LINEAR_VALID_BIT;
			}
			if ((result.relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT)) {
				v->velocityFlags |= XR_SPACE_VELOCITY_ANGULAR_VALID_BIT;
			}

			v->linearVelocity.x = result.linear_velocity.x;
			v->linearVelocity.y = result.linear_velocity.y;
			v->linearVelocity.z = result.linear_velocity.z;

			v->angularVelocity.x = result.angular_velocity.x;
			v->angularVelocity.y = result.angular_velocity.y;
			v->angularVelocity.z = result.angular_velocity.z;
		}
	}

	return ret;

hand_tracking_inactive:
	locations->isActive = XR_FALSE;

#ifdef OXR_HAVE_EXT_hand_tracking_data_source
	if (data_source_state != NULL) {
		data_source_state->isActive = XR_FALSE;
	}
#endif

	// Loop over all joints and zero flags.
	for (uint32_t i = 0; i < locations->jointCount; i++) {
		locations->jointLocations[i].locationFlags = XRT_SPACE_RELATION_BITMASK_NONE;
		if (vel) {
			vel->jointVelocities[i].velocityFlags = XRT_SPACE_RELATION_BITMASK_NONE;
		}
	}

	return ret;
}

#endif // OXR_HAVE_EXT_hand_tracking


/*
 *
 * XR_MNDX_force_feedback_curl
 *
 */

#ifdef XR_MNDX_force_feedback_curl

XrResult
oxr_hand_tracker_apply_force_feedback(struct oxr_logger *log,
                                      struct oxr_hand_tracker *hand_tracker,
                                      const XrForceFeedbackCurlApplyLocationsMNDX *locations)
{
	struct xrt_output_value result = {0};
	result.type = XRT_OUTPUT_VALUE_TYPE_FORCE_FEEDBACK;
	result.force_feedback.force_feedback_location_count = locations->locationCount;
	for (uint32_t i = 0; i < locations->locationCount; i++) {
		result.force_feedback.force_feedback[i].location =
		    (enum xrt_force_feedback_location)locations->locations[i].location;
		result.force_feedback.force_feedback[i].value = locations->locations[i].value;
	}

	const struct oxr_hand_tracking_data_source *data_sources[2] = {
	    &hand_tracker->unobstructed,
	    &hand_tracker->conforming,
	};
	for (uint32_t i = 0; i < ARRAY_SIZE(data_sources); ++i) {
		struct xrt_device *xdev = data_sources[i]->xdev;
		if (xdev) {
			xrt_result_t xret =
			    xrt_device_set_output(xdev, xr_hand_to_force_feedback_output(hand_tracker->hand), &result);
			if (xret != XRT_SUCCESS) {
				return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "xr_device_set_output failed");
			}
		}
	}

	return XR_SUCCESS;
}

#endif // XR_MNDX_force_feedback_curl
