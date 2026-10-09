// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Process-global one-shot screenshot request flag.
 * @ingroup aux_util
 *
 * A tiny thread- and signal-safe channel so anything in the server process
 * (an input driver, a signal handler, the gesture detector) can ask the
 * compositor to grab a screenshot, without taking a dependency on the
 * compositor itself. A request may optionally carry a normalised crop region.
 *
 * A request can also be *quiet*: a capture for another program (an assistant
 * looking at the view, say) rather than a photo — no shutter sound, and it
 * never lands in the screenshot folder. Quiet and normal requests are tracked
 * separately, so one never swallows the other.
 */
#pragma once

#include "xrt/xrt_compiler.h"

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * A consumed screenshot request.
 *
 * @ingroup aux_util
 */
struct u_screenshot_request
{
	//! If true, crop to [x0,y0]..[x1,y1]; else capture the full view.
	bool has_region;
	//! Normalised crop rect, [0,1], origin top-left (matches the image).
	float x0, y0, x1, y1;
	//! A capture for another program: no sound, written to the capture dir.
	bool quiet;
};

/*!
 * Request a one-shot full-view screenshot. Async-signal-safe and thread-safe.
 *
 * @ingroup aux_util
 */
void
u_screenshot_request(void);

/*!
 * Request a one-shot screenshot cropped to a normalised rect ([0,1], top-left
 * origin). Thread-safe (call from a normal thread, not a signal handler).
 *
 * @ingroup aux_util
 */
void
u_screenshot_request_rect(float x0, float y0, float x1, float y1);

/*!
 * Request a one-shot quiet full-view capture (see @ref u_screenshot_request).
 * Async-signal-safe and thread-safe.
 *
 * @ingroup aux_util
 */
void
u_screenshot_request_quiet(void);

/*!
 * Consume a pending request, returning true at most once per request and
 * filling @p out (may be NULL). Intended to be polled by the compositor. With
 * both kinds pending, the normal one comes first and the quiet one on the
 * next call.
 *
 * @ingroup aux_util
 */
bool
u_screenshot_consume_request(struct u_screenshot_request *out);


#ifdef __cplusplus
}
#endif
