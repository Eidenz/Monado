// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Process-global channel for live finger-frame gesture feedback.
 * @ingroup aux_util
 */

#include "util/u_gesture_feedback.h"

#include <stdatomic.h>


//! Published last (release) by the producer, read first (acquire) by the
//! consumer; the corner/progress data below is written before it is set.
static atomic_bool g_armed = false;
//! 4 corners * xyz, plus progress. Plain floats, ordered by the atomic above.
static float g_corners[12];
static float g_progress = 0.0f;

void
u_gesture_feedback_publish(const struct xrt_vec3 corners[4], float progress)
{
	for (int i = 0; i < 4; i++) {
		g_corners[i * 3 + 0] = corners[i].x;
		g_corners[i * 3 + 1] = corners[i].y;
		g_corners[i * 3 + 2] = corners[i].z;
	}
	g_progress = progress;
	atomic_store(&g_armed, true);
}

void
u_gesture_feedback_clear(void)
{
	atomic_store(&g_armed, false);
}

bool
u_gesture_feedback_get(struct u_gesture_feedback *out)
{
	bool armed = atomic_load(&g_armed);
	out->armed = armed;
	if (!armed) {
		return false;
	}

	out->progress = g_progress;
	for (int i = 0; i < 4; i++) {
		out->corners[i].x = g_corners[i * 3 + 0];
		out->corners[i].y = g_corners[i * 3 + 1];
		out->corners[i].z = g_corners[i * 3 + 2];
	}
	return true;
}
