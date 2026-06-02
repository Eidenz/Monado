// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Base timing source implementation.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup base
 */

#include "util/u_misc.h"

#include "b_timing_source.h"


/*!
 * @ref t_timing_event_source implementation
 */

static struct b_timing_source *
from_source(struct t_timing_event_source *ttes)
{
	return container_of(ttes, struct b_timing_source, base);
}

static int
b_timing_source_add_sink(struct t_timing_event_source *ttes, struct t_timing_event_sink *sink)
{
	struct b_timing_source *bts = from_source(ttes);

	int ret = -1;

	os_mutex_lock(&bts->mutex);
	for (size_t i = 0; i < ARRAY_SIZE(bts->sinks); i++) {
		if (bts->sinks[i] == NULL) {
			bts->sinks[i] = sink;

			// Success
			ret = 0;
			break;
		}
	}
	os_mutex_unlock(&bts->mutex);

	return ret;
}

static void
b_timing_source_remove_sink(struct t_timing_event_source *ttes, struct t_timing_event_sink *sink)
{
	struct b_timing_source *bts = from_source(ttes);

	os_mutex_lock(&bts->mutex);
	for (size_t i = 0; i < ARRAY_SIZE(bts->sinks); i++) {
		if (bts->sinks[i] == sink) {
			bts->sinks[i] = NULL;
		}
	}
	os_mutex_unlock(&bts->mutex);
}

/*!
 * @ref xrt_frame_node implementation
 */

static struct b_timing_source *
from_node(struct xrt_frame_node *node)
{
	return container_of(node, struct b_timing_source, node);
}

static void
b_timing_source_break_apart(struct xrt_frame_node *node)
{
	struct b_timing_source *bts = from_node(node);

	os_mutex_lock(&bts->mutex);
	bts->running = false;
	os_mutex_unlock(&bts->mutex);
}

static void
b_timing_source_destroy(struct xrt_frame_node *node)
{
	struct b_timing_source *bts = from_node(node);

	os_mutex_destroy(&bts->mutex);

	free(bts);
}

/*
 * Exported functions
 */

int
b_timing_source_init(struct xrt_frame_context *xfctx, struct b_timing_source **out_bts)
{
	struct b_timing_source *bts = U_TYPED_CALLOC(struct b_timing_source);
	if (bts == NULL) {
		return -1;
	}

	if (os_mutex_init(&bts->mutex) < 0) {
		free(bts);
		return -1;
	}

	// @ref t_timing_event_source
	bts->base.add_sink = b_timing_source_add_sink;
	bts->base.remove_sink = b_timing_source_remove_sink;

	// @ref xrt_frame_node
	bts->node.break_apart = b_timing_source_break_apart;
	bts->node.destroy = b_timing_source_destroy;

	bts->running = true;

	xrt_frame_context_add(xfctx, &bts->node);

	*out_bts = bts;
	return 0;
}

void
b_timing_source_push_event(struct b_timing_source *bts, const struct t_timing_event *event)
{
	os_mutex_lock(&bts->mutex);
	if (!bts->running) {
		os_mutex_unlock(&bts->mutex);
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(bts->sinks); i++) {
		struct t_timing_event_sink *sink = bts->sinks[i];

		if (sink != NULL) {
			t_timing_event_sink_push_timing_event(sink, event);
		}
	}
	os_mutex_unlock(&bts->mutex);
}
