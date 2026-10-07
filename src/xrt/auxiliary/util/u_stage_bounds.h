// Copyright 2026, Eidenz.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The stage's bounds (the play area), as a tracking driver knows them.
 * @ingroup aux_util
 *
 * One value for the whole process: a driver that knows the play area sets it
 * (steamvr_lh reads SteamVR's room setup), and the service answers apps'
 * xrGetReferenceSpaceBoundsRect(STAGE) with it.
 */

#pragma once

#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Set the stage's bounds: width along X, depth along Z, centred on its
 * origin. NULL when they aren't known.
 */
void
u_stage_bounds_set(const struct xrt_vec2 *bounds);

/*!
 * The stage's bounds, if known.
 */
bool
u_stage_bounds_get(struct xrt_vec2 *out_bounds);

#ifdef __cplusplus
}
#endif
