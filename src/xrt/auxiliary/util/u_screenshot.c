// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Process-global one-shot screenshot request flag.
 * @ingroup aux_util
 */

#include "util/u_screenshot.h"

#include <stdatomic.h>


//! Set by producers (input drivers, signal handlers, detector), cleared by the consumer.
static atomic_int g_requested = 0;
//! A pending quiet request (kept apart from g_requested so neither drops the other).
static atomic_int g_quiet_requested = 0;
static atomic_bool g_has_region = false;
//! Written before g_requested is set; read after it is observed (ordered by the atomics).
static float g_rect[4];

void
u_screenshot_request(void)
{
	atomic_store(&g_has_region, false);
	atomic_store(&g_requested, 1);
}

void
u_screenshot_request_rect(float x0, float y0, float x1, float y1)
{
	g_rect[0] = x0;
	g_rect[1] = y0;
	g_rect[2] = x1;
	g_rect[3] = y1;
	atomic_store(&g_has_region, true);
	atomic_store(&g_requested, 1);
}

void
u_screenshot_request_quiet(void)
{
	atomic_store(&g_quiet_requested, 1);
}

bool
u_screenshot_consume_request(struct u_screenshot_request *out)
{
	if (atomic_exchange(&g_requested, 0) != 0) {
		if (out != NULL) {
			out->has_region = atomic_load(&g_has_region);
			out->x0 = g_rect[0];
			out->y0 = g_rect[1];
			out->x1 = g_rect[2];
			out->y1 = g_rect[3];
			out->quiet = false;
		}
		return true;
	}
	if (atomic_exchange(&g_quiet_requested, 0) != 0) {
		if (out != NULL) {
			out->has_region = false;
			out->x0 = out->y0 = 0.0f;
			out->x1 = out->y1 = 1.0f;
			out->quiet = true;
		}
		return true;
	}
	return false;
}
