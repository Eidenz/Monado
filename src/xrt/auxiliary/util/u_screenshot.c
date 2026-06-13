// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Process-global one-shot screenshot request flag.
 * @ingroup aux_util
 */

#include "util/u_screenshot.h"

#include <stdatomic.h>


//! Set by producers (input drivers, signal handlers), cleared by the consumer.
static atomic_int g_requested = 0;

void
u_screenshot_request(void)
{
	atomic_store(&g_requested, 1);
}

bool
u_screenshot_consume_request(void)
{
	return atomic_exchange(&g_requested, 0) != 0;
}
