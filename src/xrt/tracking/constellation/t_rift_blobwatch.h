// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header defining the Rift blobwatch parameters and creation function.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "tracking/t_constellation.h"


struct t_rift_blobwatch_params
{
	//! Minimum pixel magnitude to be included in a blob at all
	uint8_t pixel_threshold;
	/*!
	 * Require at least 1 pixel over this threshold in a blob, allows for collecting fainter blobs, as long as they
	 * have a bright point somewhere, and helps to eliminate generally faint background noise
	 */
	uint8_t blob_required_threshold;
	/*!
	 * Maximum distance (in pixels) for matching a blob between frames.
	 * Matches beyond this distance are rejected, preventing newly appeared blobs
	 * from inheriting incorrect velocity from distant unrelated blobs.
	 */
	float max_match_dist;
};

/*!
 * @public @memberof t_rift_blobwatch
 */
int
t_rift_blobwatch_create(const struct t_rift_blobwatch_params *params,
                        struct xrt_frame_context *xfctx,
                        struct t_blob_sink *blob_sink,
                        struct xrt_frame_sink **out_frame_sink,
                        struct t_blobwatch **out_blobwatch);
