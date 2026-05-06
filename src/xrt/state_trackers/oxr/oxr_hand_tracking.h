// Copyright 2018-2024, Collabora, Ltd.
// Copyright 2025, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Hand tracking objects and functions.
 * @author Christoph Haag <christoph.haag@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup oxr_main
 */

#pragma once

#include "xrt/xrt_openxr_includes.h"

#include "oxr_handle_base.h"


/*
 *
 * Structs and defines.
 *
 */

struct oxr_session;

struct oxr_hand_tracking_data_source
{
	//! xrt_device backing this hand tracker
	struct xrt_device *xdev;

	//! the input name associated with this hand tracker
	enum xrt_input_name input_name;
};

static inline int
oxr_hand_tracking_data_source_cmp(const void *p1, const void *p2)
{
	const struct oxr_hand_tracking_data_source *lhs = (const struct oxr_hand_tracking_data_source *)p1;
	const struct oxr_hand_tracking_data_source *rhs = (const struct oxr_hand_tracking_data_source *)p2;
	assert(lhs && rhs);
	if (rhs->input_name < lhs->input_name)
		return -1;
	if (rhs->input_name > lhs->input_name)
		return 1;
	return 0;
}

/*!
 * A hand tracker.
 *
 * Parent type/handle is @ref oxr_instance
 *
 *
 * @obj{XrHandTrackerEXT}
 * @extends oxr_handle_base
 */
struct oxr_hand_tracker
{
	//! Common structure for things referred to by OpenXR handles.
	struct oxr_handle_base handle;

	//! Owner of this hand tracker.
	struct oxr_session *sess;

	struct oxr_hand_tracking_data_source unobstructed;
	struct oxr_hand_tracking_data_source conforming;

	/*!
	 * An ordered list of requested data-source from above options (@ref
	 * oxr_hand_tracker::[unobstructed|conforming]), ordered by
	 * @ref oxr_hand_tracker::input_name (see @ref oxr_hand_tracking_data_source_cmp)
	 *
	 * if OXR_HAVE_EXT_hand_tracking_data_source is not defined the list
	 * will contain refs to all the above options.
	 */
	struct oxr_hand_tracking_data_source *requested_sources[2];
	uint32_t requested_sources_count;

	XrHandEXT hand;
	XrHandJointSetEXT hand_joint_set;
};


/*
 *
 * Functions.
 *
 */

XrResult
oxr_hand_tracker_create(struct oxr_logger *log,
                        struct oxr_session *sess,
                        const XrHandTrackerCreateInfoEXT *createInfo,
                        struct oxr_hand_tracker **out_hand_tracker);

XrResult
oxr_hand_tracker_joints(struct oxr_logger *log,
                        struct oxr_hand_tracker *hand_tracker,
                        const XrHandJointsLocateInfoEXT *locateInfo,
                        XrHandJointLocationsEXT *locations);

XrResult
oxr_hand_tracker_apply_force_feedback(struct oxr_logger *log,
                                      struct oxr_hand_tracker *hand_tracker,
                                      const XrForceFeedbackCurlApplyLocationsMNDX *locations);
