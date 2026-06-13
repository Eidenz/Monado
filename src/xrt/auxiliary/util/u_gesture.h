// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Server-side hand-gesture detector.
 * @ingroup aux_util
 *
 * Runs in monado-service. Each tick it resolves the current left/right hand
 * devices from the system roles (so it follows hotplug), reads their hand
 * joints, and fires actions on recognised gestures. The first gesture is the
 * two-hand "finger frame" → screenshot (via @ref u_screenshot_request).
 */
#pragma once

#include "xrt/xrt_compiler.h"

#ifdef __cplusplus
extern "C" {
#endif

struct xrt_system_devices;
struct u_gesture;

/*!
 * Create and start the detector. Borrows @p xsysd (must outlive the detector).
 * Returns 0 on success, non-zero on failure (detector simply won't run).
 *
 * @public @memberof u_gesture
 */
int
u_gesture_create(struct xrt_system_devices *xsysd, struct u_gesture **out_gesture);

/*!
 * Stop the detector thread and free it. Must be called before @p xsysd (passed
 * to @ref u_gesture_create) is destroyed.
 *
 * @public @memberof u_gesture
 */
void
u_gesture_destroy(struct u_gesture **ptr_gesture);

#ifdef __cplusplus
}
#endif
