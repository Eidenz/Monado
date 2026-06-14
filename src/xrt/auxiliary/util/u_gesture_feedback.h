// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Process-global channel for live finger-frame gesture feedback.
 * @ingroup aux_util
 *
 * A tiny channel mirroring @ref u_screenshot: the gesture detector publishes,
 * while the finger-frame pose is held, the four frame corners (in tracking
 * space) plus the hold progress, and the compositor consumes them to draw an
 * in-headset rectangle previewing the region about to be captured. Decoupled
 * so the detector takes no dependency on the compositor.
 */
#pragma once

#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * A snapshot of the current frame-feedback state.
 *
 * @ingroup aux_util
 */
struct u_gesture_feedback
{
	//! True while the finger-frame pose is being held.
	bool armed;
	//! Hold progress in [0,1] (0 = just detected, 1 = about to fire).
	float progress;
	//! The four frame corners in tracking space (order irrelevant; consumer
	//! takes a per-eye bounding box). Only valid when @p armed.
	struct xrt_vec3 corners[4];
};

/*!
 * Publish the current frame corners and hold progress, marking the feedback
 * armed. Thread-safe (call from the detector thread).
 *
 * @ingroup aux_util
 */
void
u_gesture_feedback_publish(const struct xrt_vec3 corners[4], float progress);

/*!
 * Clear the feedback (mark not armed), so the compositor stops drawing.
 *
 * @ingroup aux_util
 */
void
u_gesture_feedback_clear(void);

/*!
 * Read the current feedback into @p out. Returns true iff armed (in which case
 * @p out->corners and @p out->progress are filled). Polled by the compositor.
 *
 * @ingroup aux_util
 */
bool
u_gesture_feedback_get(struct u_gesture_feedback *out);


#ifdef __cplusplus
}
#endif
