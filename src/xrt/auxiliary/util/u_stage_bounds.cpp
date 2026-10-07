// Copyright 2026, Eidenz.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The stage's bounds (the play area), as a tracking driver knows them.
 * @ingroup aux_util
 */

#include "util/u_stage_bounds.h"

#include <mutex>
#include <optional>

namespace {
std::mutex bounds_mutex;
std::optional<xrt_vec2> stage_bounds;
} // namespace

extern "C" void
u_stage_bounds_set(const struct xrt_vec2 *bounds)
{
	std::lock_guard lk(bounds_mutex);
	stage_bounds = bounds != nullptr ? std::optional<xrt_vec2>(*bounds) : std::nullopt;
}

extern "C" bool
u_stage_bounds_get(struct xrt_vec2 *out_bounds)
{
	std::lock_guard lk(bounds_mutex);
	if (!stage_bounds) {
		return false;
	}
	*out_bounds = *stage_bounds;
	return true;
}
