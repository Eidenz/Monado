// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  In-headset screenshot capture (off-thread PNG writer).
 * @ingroup comp_main
 */
#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_frame.h"
#include "os/os_threading.h"
#include "util/u_screenshot.h"


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * Off-thread screenshot writer. Host-visible readback frames (produced by
 * @ref comp_mirror_blit_to_frame) are queued here and encoded to PNG on a
 * worker thread, so the compositor thread never blocks on encode or disk I/O.
 *
 * @ingroup comp_main
 */
struct comp_screenshot
{
	struct os_thread_helper oth;

	//! Pending frame to write, protected by @ref oth. Owns one reference.
	struct xrt_frame *pending;

	//! Crop region for the pending frame, protected by @ref oth.
	struct u_screenshot_request pending_req;

	//! Directory screenshots are written to.
	char dir[512];

	//! Counter for filename uniqueness within a run.
	uint32_t counter;
};

/*!
 * Initialise and start the worker thread, and install the test trigger
 * (a SIGUSR1 handler, see @ref comp_screenshot_consume_request).
 *
 * @public @memberof comp_screenshot
 */
int
comp_screenshot_init(struct comp_screenshot *cs);

/*!
 * Returns true if a capture has been requested (e.g. via SIGUSR1) since the
 * last call, clearing the request. Cheap to poll once per frame.
 *
 * @public @memberof comp_screenshot
 */
bool
comp_screenshot_consume_request(struct comp_screenshot *cs, struct u_screenshot_request *out);

/*!
 * Hand a captured readback frame to the worker thread to be written to disk.
 * Takes ownership of the caller's reference to @p frame.
 *
 * @public @memberof comp_screenshot
 */
void
comp_screenshot_submit(struct comp_screenshot *cs, struct xrt_frame *frame, const struct u_screenshot_request *req);

/*!
 * Stop the worker thread and release resources. Must be called before the
 * readback pool that produced any queued frames is destroyed.
 *
 * @public @memberof comp_screenshot
 */
void
comp_screenshot_fini(struct comp_screenshot *cs);


#ifdef __cplusplus
}
#endif
