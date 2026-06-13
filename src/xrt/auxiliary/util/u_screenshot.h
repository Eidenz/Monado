// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Process-global one-shot screenshot request flag.
 * @ingroup aux_util
 *
 * A tiny thread- and signal-safe channel so anything in the server process
 * (an input driver, a signal handler, a future gesture detector) can ask the
 * compositor to grab a screenshot, without taking a dependency on the
 * compositor itself.
 */
#pragma once

#include "xrt/xrt_compiler.h"

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * Request a one-shot compositor screenshot. Async-signal-safe and thread-safe.
 *
 * @ingroup aux_util
 */
void
u_screenshot_request(void);

/*!
 * Consume a pending request, returning true at most once per request.
 * Intended to be polled by the compositor once per frame.
 *
 * @ingroup aux_util
 */
bool
u_screenshot_consume_request(void);


#ifdef __cplusplus
}
#endif
