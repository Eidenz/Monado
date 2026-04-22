// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Rift timing source code.
 *
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_rift
 */

#include "util/u_misc.h"

#include "rift_timing_source.h"


/*!
 * @ref t_timing_event_source implementation
 */

static struct rift_timing_source *
from_source(struct t_timing_event_source *source)
{
	return container_of(source, struct rift_timing_source, base);
}

static int
rift_timing_source_add_sink(struct t_timing_event_source *ttes, struct t_timing_event_sink *sink)
{
	struct rift_timing_source *source = from_source(ttes);

	int ret = -1;

	os_mutex_lock(&source->mutex);
	for (size_t i = 0; i < RIFT_MAX_TIMING_SINKS; i++) {
		if (source->sinks[i] == NULL) {
			source->sinks[i] = sink;

			// Success
			ret = 0;
			break;
		}
	}
	os_mutex_unlock(&source->mutex);

	return ret;
}

static void
rift_timing_source_remove_sink(struct t_timing_event_source *ttes, struct t_timing_event_sink *sink)
{
	struct rift_timing_source *source = from_source(ttes);

	os_mutex_lock(&source->mutex);
	for (size_t i = 0; i < RIFT_MAX_TIMING_SINKS; i++) {
		if (source->sinks[i] == sink) {
			source->sinks[i] = NULL;
		}
	}
	os_mutex_unlock(&source->mutex);
}

/*!
 * @ref xrt_frame_node implementation
 */

static struct rift_timing_source *
from_node(struct xrt_frame_node *node)
{
	return container_of(node, struct rift_timing_source, node);
}

static void
rift_timing_source_break_apart(struct xrt_frame_node *node)
{
	struct rift_timing_source *source = from_node(node);

	source->running = false;
}

static void
rift_timing_source_destroy(struct xrt_frame_node *node)
{
	struct rift_timing_source *source = from_node(node);

	os_mutex_destroy(&source->mutex);

	free(source);
}

/*
 * Exported functions
 */

int
rift_timing_source_init(struct xrt_frame_context *xfctx, struct rift_timing_source **out_source)
{
	struct rift_timing_source *source = U_TYPED_CALLOC(struct rift_timing_source);
	if (source == NULL) {
		return -1;
	}

	if (os_mutex_init(&source->mutex) < 0) {
		free(source);
		return -1;
	}

	source->base.add_sink = rift_timing_source_add_sink;
	source->base.remove_sink = rift_timing_source_remove_sink;

	source->node.break_apart = rift_timing_source_break_apart;
	source->node.destroy = rift_timing_source_destroy;

	source->running = true;

	xrt_frame_context_add(xfctx, &source->node);

	*out_source = source;
	return 0;
}

void
rift_timing_source_push_event(struct rift_timing_source *source, const struct t_timing_event *event)
{
	if (!source->running) {
		return;
	}

	os_mutex_lock(&source->mutex);
	for (size_t i = 0; i < RIFT_MAX_TIMING_SINKS; i++) {
		struct t_timing_event_sink *sink = source->sinks[i];

		if (sink != NULL) {
			t_timing_event_sink_push_timing_event(sink, event);
		}
	}
	os_mutex_unlock(&source->mutex);
}
