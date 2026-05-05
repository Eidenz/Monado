// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Rift timing source code.
 *
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_rift
 */

#pragma once

#include "os/os_threading.h"

#include "rift_interface.h"


struct rift_timing_source
{
	struct t_timing_event_source base;
	struct xrt_frame_node node;

	struct os_mutex mutex;

	bool running;

	struct t_timing_event_sink *sinks[RIFT_MAX_TIMING_SINKS];
};

int
rift_timing_source_init(struct xrt_frame_context *xfctx, struct rift_timing_source **out_source);

void
rift_timing_source_push_event(struct rift_timing_source *source, const struct t_timing_event *event);
