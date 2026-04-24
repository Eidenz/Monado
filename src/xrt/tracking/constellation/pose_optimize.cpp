// Copyright 2015, Philipp Zabel
// Copyright 2020-2023, Jan Schmidt
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

#include "util/u_logging.h"

#include "tracking/t_camera_models.h"
#include "tracking/t_constellation.h"

#include "math/m_api.h"

#include "camera_model.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if CV_MAJOR_VERSION >= 4
#include <opencv2/calib3d/calib3d_c.h>
#endif

#include <iostream>
#include <stdio.h>

#include "pose_optimize.h"


static void
undistort_blob_points(std::vector<cv::Point2f> in_points,
                      std::vector<cv::Point2f> &out_points,
                      struct camera_model *calib)
{
	for (size_t i = 0; i < in_points.size(); i++) {
		t_camera_models_undistort(&calib->calib, in_points[i].x, in_points[i].y, &out_points[i].x,
		                          &out_points[i].y);
	}
}

bool
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
	int i, j;
	int num_leds = 0;
	uint64_t taken = 0;
	int flags = cv::SOLVEPNP_SQPNP;
	cv::Mat inliers;
	int iterationsCount = 100;
	float confidence = 0.99;
	cv::Mat dummyK = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat dummyD = cv::Mat::zeros(4, 1, CV_64FC1);
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);

	tvec.at<double>(0) = pose->position.x;
	tvec.at<double>(1) = pose->position.y;
	tvec.at<double>(2) = pose->position.z;

	// convert the input pose to a rotation vector for the PnP solver
	rvec.at<double>(0) = pose->orientation.x;
	rvec.at<double>(1) = pose->orientation.y;
	rvec.at<double>(2) = pose->orientation.z;
	rvec = 2 * acosf(pose->orientation.w) * rvec / cv::norm(rvec);

	// std::cout << "R = " << R << ", rvec = " << rvec << std::endl;

	// Count identified leds
	for (i = 0; i < num_blobs; i++) {
		t_constellation_device_id_t blob_device_id = blobs[i].matched_device_id;
		t_constellation_led_id_it blob_led_id = blobs[i].matched_device_led_id;

		// int led_id = blobs[i].led_id;
		if (blob_device_id != device_id) {
			continue; /* invalid or LED id for another object */
		}

		if (taken & (1ULL << blob_led_id)) {
			continue;
		}

		taken |= (1ULL << blob_led_id);
		num_leds++;
	}
	if (num_leds_out)
		*num_leds_out = num_leds;

	if (num_leds < 4) {
		U_LOG_IFL_D(log_level, "Not enough LEDs for PnP: %d", num_leds);
		return false;
	}

	std::vector<cv::Point3f> list_points3d(num_leds);
	std::vector<cv::Point2f> list_points2d(num_leds);
	std::vector<cv::Point2f> list_points2d_undistorted(num_leds);

	taken = 0;
	for (i = 0, j = 0; i < num_blobs && j < num_leds; i++) {
		t_constellation_device_id_t blob_device_id = blobs[i].matched_device_id;
		t_constellation_led_id_it blob_led_id = blobs[i].matched_device_led_id;

		// invalid or LED id for another object
		if (blob_device_id != device_id) {
			continue;
		}

		if (taken & (1ULL << blob_led_id)) {
			continue;
		}

		taken |= (1ULL << blob_led_id);
		list_points3d[j].x = leds_model->leds[blob_led_id].position.x;
		list_points3d[j].y = leds_model->leds[blob_led_id].position.y;
		list_points3d[j].z = leds_model->leds[blob_led_id].position.z;
		list_points2d[j].x = blobs[i].center.x;
		list_points2d[j].y = blobs[i].center.y;
		j++;

		U_LOG_IFL_D(log_level, "LED %d at %f,%f (3D %f %f %f)", blob_led_id, blobs[i].center.x,
		            blobs[i].center.y, leds_model->leds[blob_led_id].position.x,
		            leds_model->leds[blob_led_id].position.y, leds_model->leds[blob_led_id].position.z);
	}

	num_leds = j;
	if (num_leds < 4) {
		U_LOG_IFL_D(log_level, "Not enough unique LEDs for PnP: %d", num_leds);
		return false;
	}
	list_points3d.resize(num_leds);
	list_points2d.resize(num_leds);
	list_points2d_undistorted.resize(num_leds);

	// we undistort the image points manually before passing them to the PnpRansac solver
	// and we give the solver identity camera + null distortion matrices
	undistort_blob_points(list_points2d, list_points2d_undistorted, calib);

	// 3 pixel reprojection threshold
	float reprojectionError = 3.0 / calib->calib.fx;

#if 1
	if (!cv::solvePnPRansac(list_points3d, list_points2d_undistorted, dummyK, dummyD, rvec, tvec, false,
	                        iterationsCount, reprojectionError, confidence, inliers, flags)) {
		return false;
	}
#else
	cv::solvePnPRefineLM(list_points3d, list_points2d_undistorted, dummyK, dummyD, rvec, tvec);
#endif

	if (num_inliers)
		*num_inliers = inliers.rows;

	struct xrt_vec3 v;
	double angle = sqrt(rvec.dot(rvec));
	double inorm = 1.0f / angle;

	v.x = rvec.at<double>(0) * inorm;
	v.y = rvec.at<double>(1) * inorm;
	v.z = rvec.at<double>(2) * inorm;
	math_quat_from_angle_vector(angle, &v, &pose->orientation);
	pose->position.x = tvec.at<double>(0);
	pose->position.y = tvec.at<double>(1);
	pose->position.z = tvec.at<double>(2);

	U_LOG_IFL_T(log_level, "Got PnP pose quat %f %f %f %f  pos %f %f %f", pose->orientation.x, pose->orientation.y,
	            pose->orientation.z, pose->orientation.w, pose->position.x, pose->position.y, pose->position.z);

	return true;
}
