// Copyright 2025, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Code to parse and handle the Vive Pro 2 configuration data.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_vp2
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


enum vp2_distortion_model
{
	VP2_DISTORTION_MODEL_INVALID = 0,
	VP2_DISTORTION_MODEL_TRADITIONAL_SIMPLE = 1,
	VP2_DISTORTION_MODEL_TRADITIONAL_WITH_TANGENTIAL = 2,
	VP2_DISTORTION_MODEL_NON_MODEL_SVR = 3,
	VP2_DISTORTION_MODEL_RATIONAL = 4,
	VP2_DISTORTION_MODEL_SECTIONAL = 5,
	VP2_DISTORTION_MODEL_TANGENTIAL_WEIGHT = 6,
	VP2_DISTORTION_MODEL_RADIAL_TANGENTIAL_PRISM = 7,
	VP2_DISTORTION_MODEL_PRISM_WITH_PROGRESSIVE = 8,
	VP2_DISTORTION_MODEL_STRENGTHEN_RADIAL = 9,
	VP2_DISTORTION_MODEL_STRENGTHEN = 10,
	VP2_DISTORTION_MODEL_STRENGTHEN_HIGH_ORDER = 11,
	VP2_DISTORTION_MODEL_WVR_RADIAL = 12,
	VP2_DISTORTION_MODEL_RADIAL_ROTATE_MODIFY = 13,
};

struct vp2_eye_coefficients
{
	double k[13]; // radial
	double p[6];  // tangential
	double s[4];  // prism
};

struct vp2_wvr_coefficients
{
	double k[6];
};

struct vp2_modify_coefficients
{
	double k[11];
	double theta;
};

struct vp2_distortion_version
{
	uint8_t major;
	uint8_t minor;
};

struct vp2_warp_parameters
{
	struct xrt_matrix_3x3 post;
	struct xrt_matrix_3x3 pre;
	float max_radius;
};

struct vp2_eye_distortion
{
	struct xrt_vec2 center;
	enum vp2_distortion_model model;

	struct vp2_eye_coefficients coeffecients[3];

	struct vp2_wvr_coefficients wvr[3];

	struct vp2_modify_coefficients modify; // Only contains blue

	double enlarge_ratio;
	double grow_for_undistort[4];

	struct xrt_matrix_3x3 intrinsics[2];

	struct xrt_vec2_i32 resolution;
	double scale;
	double scale_ratio;

	double normalized_radius;

	struct vp2_distortion_version version;

	struct vp2_warp_parameters warp;
};

struct vp2_config
{
	struct
	{
		uint32_t eye_target_width_in_pixels;
		uint32_t eye_target_height_in_pixels;
	} device;

	uint32_t direct_mode_edid_pid;
	uint32_t direct_mode_edid_vid;

	struct
	{
		struct vp2_eye_distortion eyes[2];

		struct xrt_vec2_i32 resolution; // The resolution to use for distorting
	} lens_correction;
};

enum vp2_distortion_model
vp2_string_to_distortion_model(const char *model_str);

bool
vp2_config_parse(const char *config_data, size_t config_size, struct vp2_config *out_config);

bool
vp2_distort(struct vp2_config *config, int eye, const struct xrt_vec2 *in, struct xrt_uv_triplet *out_result);

void
vp2_get_fov(struct vp2_config *config, int eye, struct xrt_fov *out_fov);


#ifdef __cplusplus
}
#endif
