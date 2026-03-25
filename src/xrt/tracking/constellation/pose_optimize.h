// Copyright 2015, Philipp Zabel
// Copyright 2020-2023
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  RANSAC PnP pose refinement
 * @author Philipp Zabel <philipp.zabel@gmail.com>
 * @author Jan Schmidt <jan@centricular.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_config_have.h"
#include "xrt/xrt_defines.h"

#include "tracking/t_constellation.h"

#include "camera_model.h"


#ifdef __cplusplus
extern "C" {
#endif

#ifdef XRT_HAVE_OPENCV
bool
ransac_pnp_pose(enum u_logging_level log_level,
                struct xrt_pose *pose,
                struct t_blob *blobs,
                int num_blobs,
                struct t_constellation_tracker_led_model *leds_model,
                t_constellation_device_id_t device_id,
                struct camera_model *calib,
                int *num_leds_out,
                int *num_inliers);

#else
static inline bool
ransac_pnp_pose(enum u_logging_level log_level,
                struct xrt_pose *pose,
                struct t_blob *blobs,
                int num_blobs,
                struct t_constellation_tracker_led_model *leds_model,
                t_constellation_device_id_t device_id,
                struct camera_model *calib,
                int *num_leds_out,
                int *num_inliers)
{
	(void)log_level;
	(void)pose;
	(void)blobs;
	(void)num_blobs;
	(void)leds_model;
	(void)device_id;
	(void)calib;
	(void)num_leds_out;
	(void)num_inliers;

	return false;
}
#endif /* HAVE_OPENCV */

#ifdef __cplusplus
}
#endif
