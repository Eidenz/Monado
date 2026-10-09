// Copyright 2015, Philipp Zabel
// Copyright 2020-2023, Jan Schmidt
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PnP pose optimization using ceres.
 * @author Philipp Zabel <philipp.zabel@gmail.com>
 * @author Jan Schmidt <jan@centricular.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#include "util/u_logging.h"

#include "tracking/t_camera_models.hpp"
#include "tracking/t_constellation.h"

#include "math/m_api.h"
#include "math/m_eigen_interop.hpp"
#include "math/m_quatexpmap.hpp"
#include "math/m_quatexpmap_bigceres.hpp"

#include "constellation/camera_model.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "ceres/autodiff_cost_function.h"
#include "ceres/problem.h"
#include "ceres/solver.h"

#include "pose_optimize.hpp"
#include "ransac.hpp"
#include "math.hpp"

#include <iostream>
#include <stdio.h>


namespace {

//! Optimize directly on the pre-undistorted points, don't compute residuals in distorted pixel space.
constexpr bool kOptimizeUndistortedPoints = false;
//! Run a RANSAC inlier optimization before the final optimization.
constexpr bool kRunRansac = true;
//! Print debug info about the residuals as we're computing them.
constexpr bool kResidualDebugPrint = false;

using namespace xrt::auxiliary::math;
using namespace xrt::auxiliary::tracking::camera_models;
using namespace xrt::tracking::constellation;
using namespace xrt::tracking::constellation::optimizer;

// @todo tune this number
#define BAD_COVARIANCE_MATRIX (PoseStateCovarianceMatrix::Identity() * 1e6)
// Matching whitener for BAD_COVARIANCE_MATRIX, sqrt(1/1e6). A failed covariance stops mattering rather than
// contributing a full-strength bogus factor.
#define BAD_WHITENING_MATRIX (PoseStateCovarianceMatrix::Identity() * 1e-3)

template <typename T> struct Pose
{
public: // Fields
	Eigen::Vector3<T> translation;
	Eigen::Quaternion<T> rotation;

public: // Methods
	Pose(Eigen::Vector3<T> translation, Eigen::Quaternion<T> rotation)
	    : translation(translation), rotation(rotation)
	{}

	//! Seeds a Pose from a translation.
	Pose(const xrt_pose &pose)
	{
		if constexpr (std::is_same_v<T, double>) {
			// Seed the translation
			this->translation.x() = pose.position.x;
			this->translation.y() = pose.position.y;
			this->translation.z() = pose.position.z;

			// Seed the rotation
			this->rotation.x() = pose.orientation.x;
			this->rotation.y() = pose.orientation.y;
			this->rotation.z() = pose.orientation.z;
			this->rotation.w() = pose.orientation.w;

			this->rotation.normalize();
		} else {
			// Seed the translation
			this->translation.x() = T(pose.position.x, static_cast<int>(PoseStateIndex::PosX));
			this->translation.y() = T(pose.position.y, static_cast<int>(PoseStateIndex::PosY));
			this->translation.z() = T(pose.position.z, static_cast<int>(PoseStateIndex::PosZ));

			// Seed the rotation
			this->rotation.x() = T(pose.orientation.x, static_cast<int>(PoseStateIndex::RotX));
			this->rotation.y() = T(pose.orientation.y, static_cast<int>(PoseStateIndex::RotY));
			this->rotation.z() = T(pose.orientation.z, static_cast<int>(PoseStateIndex::RotZ));
			this->rotation.w() = T(pose.orientation.w, static_cast<int>(PoseStateIndex::RotW));

			this->rotation.normalize();
		}
	}

	//! Packs the Pose into a solver parameter vector.
	Eigen::Vector<T, kPoseStateSize>
	pack() const
	{
		Eigen::Vector<T, kPoseStateSize> solver_params;
		solver_params.template segment<3>(static_cast<int>(PoseStateIndex::PosX)) = this->translation;
		solver_params.template segment<4>(static_cast<int>(PoseStateIndex::RotX)) = this->rotation.coeffs();
		return solver_params;
	}

	//! Unpacks a solver parameter vector into an xrt_pose.
	static xrt_pose
	unpackToPose(const Eigen::Vector<double, kPoseStateSize> &parameters)
	{
		xrt_pose pose;
		map_vec3(pose.position) = parameters.segment<3>(static_cast<int>(PoseStateIndex::PosX)).cast<float>();
		map_quat(pose.orientation) =
		    parameters.segment<4>(static_cast<int>(PoseStateIndex::RotX)).cast<float>();

		return pose;
	}

	template <typename Derived>
	static Pose<T>
	unpack(Eigen::Ref<const Eigen::Vector<Derived, kPoseStateSize>> parameters)
	{
		const auto translation =
		    parameters.template segment<3>(static_cast<int>(PoseStateIndex::PosX)).template cast<T>();
		const auto rotation = Eigen::Quaternion<T>(
		    parameters.template segment<4>(static_cast<int>(PoseStateIndex::RotX)).template cast<T>());

		return Pose<T>(translation, rotation);
	}
};

template <typename T>
void
project_led(const t_camera_model_params &params,
            const Eigen::Quaternion<T> &Q_cam_model,
            const Eigen::Vector3<T> &T_cam_model,
            const Eigen::Vector3<T> &T_model_led,
            Eigen::Vector2<T> &out_projected_point,
            T *led_depth_m)
{
	// Rotate the point around the pose's local frame.
	Eigen::Vector3<T> T_cam_led = Q_cam_model * T_model_led;
	// Translate the point into the camera frame.
	T_cam_led += T_cam_model;

	if (led_depth_m != nullptr) {
		*led_depth_m = T_cam_led.z();
	}

	Eigen::Vector2<T> projected_point = {T(1e6), T(1e6)};

	if (T_cam_led.z() <= T(0)) {
		// The point is behind the camera, so we can't project it. Return a large value to indicate this.
		out_projected_point = projected_point;
		return;
	}

	if constexpr (kOptimizeUndistortedPoints) {
		// We're optimizing directly on undistorted points, so we don't project the points through the
		// distortion model. Instead, we just use the normalized camera coordinates.
		projected_point.x() = T_cam_led.x() / T_cam_led.z();
		projected_point.y() = T_cam_led.y() / T_cam_led.z();
	} else {
		// Project the point into the camera frame, ignore error, since even if the projection fails, it
		// still always returns *some* value with possibly meaningful derivatives.
		// (unless in case of memory corruption which no derivative 1e6 is *fine*).
		(void)project(params,               //
		              T_cam_led.x(),        //
		              T_cam_led.y(),        //
		              T_cam_led.z(),        //
		              projected_point.x(),  //
		              projected_point.y()); //
	}

	out_projected_point = projected_point;
}

template <typename T, typename Derived>
void
computeSingleResidual(const t_camera_model_params &params,
                      const Eigen::Vector3<T> &T_cam_model,
                      const Eigen::Quaternion<T> &Q_cam_model,
                      const Eigen::Vector2<T> &blob_position_2d,
                      const Eigen::Vector3<T> &T_model_led,
                      Eigen::MatrixBase<Derived> &residual)
{
	Eigen::Vector2<T> predicted_point;
	T led_depth;
	project_led<T>(params,          //
	               Q_cam_model,     //
	               T_cam_model,     //
	               T_model_led,     //
	               predicted_point, //
	               &led_depth);     //

	if constexpr (kResidualDebugPrint) {
		if constexpr (std::is_same_v<T, double>) {
			U_LOG_W("meas=(%f,%f)", blob_position_2d.x(), blob_position_2d.y());
			U_LOG_W("pred=(%f,%f)", predicted_point.x(), predicted_point.y());

			U_LOG_W("pose=(%f,%f,%f), (%f,%f,%f)", T_cam_model.x(), T_cam_model.y(), T_cam_model.z(),
			        Q_cam_model.x(), Q_cam_model.y(), Q_cam_model.z());

			if (std::isnan(predicted_point.x()) || std::isnan(predicted_point.y())) {
				U_LOG_W("Projected point is NaN, led_depth=%f", led_depth);

				// XRT_DEBUGBREAK();
			}
		} else {
			U_LOG_W("meas=(%f,%f)", blob_position_2d.x().a, blob_position_2d.y().a);
			U_LOG_W("pred=(%f,%f)", predicted_point.x().a, predicted_point.y().a);

			U_LOG_W("pose=(%f,%f,%f), (%f,%f,%f)", T_cam_model.x().a, T_cam_model.y().a, T_cam_model.z().a,
			        Q_cam_model.x().a, Q_cam_model.y().a, Q_cam_model.z().a);

			if (std::isnan(predicted_point.x().a) || std::isnan(predicted_point.y().a)) {
				U_LOG_W("Projected point is NaN, led_depth=%f", led_depth.a);

				// XRT_DEBUGBREAK();
			}
		}
	}

	// Compute the residual
	residual = predicted_point - blob_position_2d;

	// Provide a smooth penalty for points behind the camera, since if we just set a large residual,
	// the solver will have no derivative to try to work back from. This solution penalizes bad
	// poses, while pushing the optimizer towards the correct solution (no LEDs behind the camera).
	if (led_depth < T(0.001)) {
		residual += Eigen::Vector2<T>::Constant(T(1000) * (T(0.001) - T(led_depth)));
	}
}

struct PnPOptimizeCostFunctor
{
	uint32_t num_leds;
	std::vector<Eigen::Vector2f> blob_positions;
	std::vector<Eigen::Vector3f> T_model_leds;
	const t_camera_model_params &params;

	template <typename T>
	bool
	operator()(const T *const parameters, T *residuals) const
	{
		const Pose<T> pose =
		    Pose<T>::template unpack<T>(Eigen::Map<const Eigen::Vector<T, kPoseStateSize>>(parameters));

		for (uint32_t i = 0; i < this->num_leds; i++) {
			const Eigen::Vector2<T> blob_position_2d = this->blob_positions[i].cast<T>();
			const Eigen::Vector3<T> T_model_led = this->T_model_leds[i].cast<T>();

			Eigen::Map<Eigen::Vector2<T>> residual(&residuals[i * 2]);

			computeSingleResidual(this->params,     //
			                      pose.translation, //
			                      pose.rotation,    //
			                      blob_position_2d, //
			                      T_model_led,      //
			                      residual);        //
		}

		return true;
	}
};

typedef ceres::AutoDiffCostFunction<PnPOptimizeCostFunctor, ceres::DYNAMIC, kPoseStateSize> CostFunction;

/*!
 * Finds a LED in the model by its id.
 *
 * A LED's id is an opaque identifier chosen by the driver, not its position in the model: drivers are free to
 * derive it from device-specific numbering (the Rift uses the headset's own LED indices, which include a slot for
 * the IMU). So a blob's matched_device_led_id has to be resolved through here rather than used as an index.
 *
 * @return The index of the LED in the model, or -1 when no LED carries that id.
 */
static int32_t
findLedIndexById(const t_constellation_tracker_led_model *leds_model, t_constellation_led_id_it led_id)
{
	for (size_t i = 0; i < leds_model->led_count; i++) {
		if (leds_model->leds[i].id == led_id) {
			return static_cast<int32_t>(i);
		}
	}

	return -1;
}

uint32_t
pickLabelledBlobs(t_blob *blobs,
                  uint32_t num_blobs,
                  t_constellation_tracker_led_model *leds_model,
                  t_constellation_device_id_t device_id,
                  std::vector<Eigen::Vector2f> &points2d,
                  std::vector<Eigen::Vector3f> &points3d,
                  std::vector<t_constellation_led_id_it> &led_ids,
                  std::vector<Eigen::Vector3f> &normals3d,
                  std::vector<t_blob *> &inlier_blobs)
{
	uint64_t taken = 0;
	uint32_t num_leds = 0;
	for (uint32_t i = 0; i < num_blobs; i++) {
		t_constellation_device_id_t blob_device_id = blobs[i].matched_device_id;
		t_constellation_led_id_it blob_led_id = blobs[i].matched_device_led_id;

		// Invalid or LED id for another object
		if (blob_device_id != device_id) {
			continue;
		}

		// Labelled with an id this model doesn't have
		int32_t led_index = findLedIndexById(leds_model, blob_led_id);
		if (led_index < 0) {
			continue;
		}

		// If this LED is already taken by another blob, skip it. Keyed on the model index rather than the id,
		// since the mask only has room for MAX_OBJECT_LEDS entries and ids need not fit in that range.
		if (taken & (1ULL << led_index)) {
			continue;
		}

		// Mark this LED as taken by this blob.
		taken |= (1ULL << led_index);
		num_leds++;
	}

	points2d.reserve(num_leds);
	points3d.reserve(num_leds);
	led_ids.reserve(num_leds);
	normals3d.reserve(num_leds);
	inlier_blobs.reserve(num_leds);

	taken = 0;
	num_leds = 0;
	for (uint32_t i = 0; i < num_blobs; i++) {
		t_constellation_device_id_t blob_device_id = blobs[i].matched_device_id;
		t_constellation_led_id_it blob_led_id = blobs[i].matched_device_led_id;

		// Invalid or LED id for another object
		if (blob_device_id != device_id) {
			continue;
		}

		// Labelled with an id this model doesn't have
		int32_t led_index = findLedIndexById(leds_model, blob_led_id);
		if (led_index < 0) {
			continue;
		}

		// If this LED is already taken by another blob, skip it.
		if (taken & (1ULL << led_index)) {
			continue;
		}

		t_constellation_tracker_led &led = leds_model->leds[led_index];

		// We have a valid LED, add it to the optimization.
		points2d.push_back({blobs[i].center.x, blobs[i].center.y});
		points3d.push_back({led.position.x, led.position.y, led.position.z});
		led_ids.push_back(led.id);
		normals3d.push_back({led.normal.x, led.normal.y, led.normal.z});
		inlier_blobs.push_back(&blobs[i]);

		// Mark this LED as taken by this blob.
		taken |= (1ULL << led_index);
		num_leds++;
	}

	return num_leds;
}

void
conditionPoints(const t_camera_model_params &params, std::vector<Eigen::Vector2f> &points2d)
{
	// Undistort the points before passing them to the optimizer.
	if constexpr (kOptimizeUndistortedPoints) {
		// undistort all 2d points
		for (size_t i = 0; i < points2d.size(); i++) {
			Eigen::Vector2f &p = points2d[i];
			float x_undistorted = 0.0f, y_undistorted = 0.0f;
			undistort<float>(params, p.x(), p.y(), x_undistorted, y_undistorted);
			p = Eigen::Vector2f(x_undistorted, y_undistorted);
		}
	}
}

void
setupProblem(Eigen::Vector<double, kPoseStateSize> &solver_params,
             PoseManifold &pose_manifold,
             CostFunction &cost_function,
             ceres::Solver::Options &out_options,
             ceres::Problem &out_problem)
{
	ceres::Problem::Options problem_options{};
	problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	problem_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

	ceres::Problem problem(problem_options);
	problem.AddParameterBlock(solver_params.data(), kPoseStateSize, &pose_manifold);
	problem.AddResidualBlock(&cost_function, nullptr, solver_params.data());

	ceres::Solver::Options options{};
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
	options.num_threads = 1;
	options.logging_type = ceres::SILENT;
	options.minimizer_progress_to_stdout = false;

	out_options = std::move(options);
	out_problem = std::move(problem);
}

//! Blob centroids are never localized better than this, in pixels.
constexpr double kMinResidualSigmaPixels = 0.5;

bool
computeCovariance(uint32_t num_residuals,
                  double rss,
                  const PoseStateCovarianceMatrix &H,
                  Eigen::Ref<PoseStateCovarianceMatrix> out_covariance,
                  Eigen::Ref<PoseStateCovarianceMatrix> out_whitening)
{
	assert(num_residuals > kPoseCovarianceSize);

	const double sigma2 = rss / static_cast<double>(num_residuals - kPoseCovarianceSize);
	const double sigma = std::max(std::sqrt(sigma2), kMinResidualSigmaPixels);

	Eigen::LLT<PoseStateCovarianceMatrix> llt(H);
	if (llt.info() != Eigen::Success) {
		return false;
	}

	// H = L L^T, so sigma * L^-T is a square root of the covariance and its inverse, L^T / sigma, is the
	// whitener: the square root of the information matrix.
	const PoseStateCovarianceMatrix L = llt.matrixL();
	out_whitening = L.transpose() / sigma;

	out_covariance = (sigma * sigma) * llt.solve(PoseStateCovarianceMatrix::Identity());

	assert((out_whitening.transpose() * out_whitening * out_covariance).isIdentity(1e-6));

	if (!out_whitening.allFinite() || !out_covariance.allFinite()) {
		return false;
	}

	return true;
}

bool
computeProblemHessian(ceres::Problem &problem,
                      uint32_t &out_num_residuals,
                      double &out_rss,
                      PoseStateCovarianceMatrix &out_H)
{
	ceres::CRSMatrix jacobian_crs;
	double cost = 0.0;

	if (!problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr, nullptr, &jacobian_crs)) {
		return false;
	}

	assert(jacobian_crs.num_cols == kPoseCovarianceSize);

	Eigen::MatrixXd J = Eigen::MatrixXd::Zero(jacobian_crs.num_rows, jacobian_crs.num_cols);
	for (int row = 0; row < jacobian_crs.num_rows; row++) {
		for (int i = jacobian_crs.rows[row]; i < jacobian_crs.rows[row + 1]; i++) {
			J(row, jacobian_crs.cols[i]) = jacobian_crs.values[i];
		}
	}

	out_num_residuals = static_cast<uint32_t>(jacobian_crs.num_rows);
	// Ceres' cost is 0.5 * ||r||^2, so twice it is the residual sum of squares computeCovariance wants.
	out_rss = cost * 2.0;
	out_H = J.transpose() * J;

	return true;
}

}; // namespace

namespace xrt::tracking::constellation::optimizer {

bool
optimizePose(u_logging_level log_level,
             bool deterministic,
             const t_camera_model_params &params,
             xrt_pose init_pose,
             t_blob *blobs,
             uint32_t num_blobs,
             t_constellation_tracker_led_model *leds_model,
             t_constellation_device_id_t device_id,
             xrt_pose &out_pose,
             RawPoseCovarianceMatrix out_covariance,
             RawPoseCovarianceMatrix out_whitening)
{
	std::vector<Eigen::Vector2f> points2d;
	std::vector<Eigen::Vector3f> points3d;
	std::vector<t_constellation_led_id_it> led_ids;
	std::vector<Eigen::Vector3f> normals3d;
	std::vector<t_blob *> inlier_blobs;

	uint32_t num_leds = pickLabelledBlobs(blobs,         //
	                                      num_blobs,     //
	                                      leds_model,    //
	                                      device_id,     //
	                                      points2d,      //
	                                      points3d,      //
	                                      led_ids,       //
	                                      normals3d,     //
	                                      inlier_blobs); //

	// We can't do an optimization with so few LEDs.
	if (num_leds < 4) {
		U_LOG_IFL_D(log_level, "Not enough LEDs for PnP optimization: %d", num_leds);
		return false;
	}

	// We only should run RANSAC if we have more than four labelled LEDs (and it's enabled)
	bool run_ransac = num_leds > 4 && kRunRansac;

	if (run_ransac) {
		std::vector<size_t> ransac_inlier_indices;
		xrt_pose computed_ransac_pose;

		bool success = ransacPose(deterministic,         //
		                          params,                //
		                          inlier_blobs,          //
		                          points3d,              //
		                          normals3d,             //
		                          ransac_inlier_indices, //
		                          computed_ransac_pose); //
		if (success) {
			init_pose = computed_ransac_pose;

			uint32_t original_num_leds = num_leds;

			// Iterate over all input inlier blobs and mark them as unlabelled (outliers)
			for (auto blob : inlier_blobs) {
				blob->matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
				blob->matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
			}

			// Update the points2d and points3d to only include the inliers from RANSAC.
			std::vector<Eigen::Vector2f> inlier_points2d;
			inlier_points2d.reserve(ransac_inlier_indices.size());
			std::vector<Eigen::Vector3f> inlier_points3d;
			inlier_points3d.reserve(ransac_inlier_indices.size());

			for (size_t index : ransac_inlier_indices) {
				inlier_points2d.push_back(points2d[index]);
				inlier_points3d.push_back(points3d[index]);

				auto inlier_blob = inlier_blobs[index];

				// Mark the blob as an inlier
				inlier_blob->matched_device_id = device_id;
				inlier_blob->matched_device_led_id = led_ids[index];
			}

			points2d = std::move(inlier_points2d);
			points3d = std::move(inlier_points3d);
			num_leds = static_cast<uint32_t>(points2d.size());

			U_LOG_IFL_D(log_level, "RANSAC found a valid pose for device %d with %d LEDs (originally %d)",
			            device_id, num_leds, original_num_leds);
		} else {
			U_LOG_IFL_D(log_level, "RANSAC failed for device %d with %d LEDs", device_id, num_leds);
		}
	}

	// Assert that RANSAC hasn't put us below 4 LEDs
	assert(num_leds >= 4);

	conditionPoints(params, points2d);

	PnPOptimizeCostFunctor optimize_cost_functor = {
	    .num_leds = num_leds,
	    .blob_positions = points2d,
	    .T_model_leds = points3d,
	    .params = params,
	};

	Eigen::Vector<double, kPoseStateSize> solver_params = Pose<double>(init_pose).pack();

	CostFunction cost_function = {
	    &optimize_cost_functor,
	    static_cast<int>(num_leds * 2),
	    ceres::DO_NOT_TAKE_OWNERSHIP,
	};
	PoseManifold pose_manifold;

	ceres::Solver::Options options;
	ceres::Problem problem;
	setupProblem(solver_params, pose_manifold, cost_function, options, problem);

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	switch (summary.termination_type) {
	case ceres::CONVERGENCE:
	case ceres::USER_SUCCESS:
		U_LOG_IFL_D(log_level, "PnP optimization converged for device %d with %d LEDs (%s)", device_id,
		            num_leds, summary.message.c_str());
		break;
	case ceres::NO_CONVERGENCE:
		U_LOG_IFL_D(log_level, "PnP optimization hit max iterations (%d) for device %d with %d LEDs",
		            options.max_num_iterations, device_id, num_leds);
		break;
	default:
		U_LOG_IFL_E(log_level, "PnP optimization failed for device %d with %d LEDs: %s", device_id, num_leds,
		            summary.message.c_str());
		return false;
	}
	U_LOG_IFL_D(log_level, "Initial cost: %f\tFinal cost: %f", summary.initial_cost, summary.final_cost);

	if (!solver_params.allFinite() || !summary.IsSolutionUsable()) {
		return false;
	}

	out_pose = Pose<double>::unpackToPose(solver_params);

	if (out_covariance != nullptr) {
		uint32_t num_residuals;
		double rss;
		PoseStateCovarianceMatrix H;

		Eigen::Map<PoseStateCovarianceMatrix> covariance(out_covariance);
		Eigen::Map<PoseStateCovarianceMatrix> whitening(out_whitening);
		if (!computeProblemHessian(problem, num_residuals, rss, H) ||           //
		    !computeCovariance(num_residuals, rss, H, covariance, whitening)) { //
			covariance = BAD_COVARIANCE_MATRIX;
			whitening = BAD_WHITENING_MATRIX;
		}
	}

	return true;
}

void
computePoseCovariance(u_logging_level log_level,
                      const t_camera_model_params &params,
                      xrt_pose init_pose,
                      t_blob *blobs,
                      uint32_t num_blobs,
                      t_constellation_tracker_led_model *leds_model,
                      t_constellation_device_id_t device_id,
                      RawPoseCovarianceMatrix out_covariance,
                      RawPoseCovarianceMatrix out_whitening)
{
	Eigen::Map<PoseStateCovarianceMatrix> covariance(out_covariance);
	Eigen::Map<PoseStateCovarianceMatrix> whitening(out_whitening);

	std::vector<Eigen::Vector2f> points2d;
	std::vector<Eigen::Vector3f> points3d;
	std::vector<t_constellation_led_id_it> led_ids;
	std::vector<Eigen::Vector3f> normals3d;
	std::vector<t_blob *> inlier_blobs;

	uint32_t num_leds = pickLabelledBlobs(blobs,         //
	                                      num_blobs,     //
	                                      leds_model,    //
	                                      device_id,     //
	                                      points2d,      //
	                                      points3d,      //
	                                      led_ids,       //
	                                      normals3d,     //
	                                      inlier_blobs); //

	// sigma2 = rss / (num_residuals - kPoseCovarianceSize), if num_residuals is less than 8 (num_leds * 2), then we
	// can't compute a covariance matrix, so we return a bad covariance matrix.
	if (num_leds < 4) {
		covariance = BAD_COVARIANCE_MATRIX;
		whitening = BAD_WHITENING_MATRIX;
		return;
	}

	conditionPoints(params, points2d);

	PnPOptimizeCostFunctor optimize_cost_functor = {
	    .num_leds = num_leds,
	    .blob_positions = points2d,
	    .T_model_leds = points3d,
	    .params = params,
	};

	Eigen::Vector<double, kPoseStateSize> solver_params = Pose<double>(init_pose).pack();

	CostFunction cost_function = {
	    &optimize_cost_functor,
	    static_cast<int>(num_leds * 2),
	    ceres::DO_NOT_TAKE_OWNERSHIP,
	};
	PoseManifold pose_manifold;

	ceres::Solver::Options options;
	ceres::Problem problem;
	setupProblem(solver_params, pose_manifold, cost_function, options, problem);

	uint32_t num_residuals;
	double rss;
	PoseStateCovarianceMatrix H;

	if (!computeProblemHessian(problem, num_residuals, rss, H) ||           //
	    !computeCovariance(num_residuals, rss, H, covariance, whitening)) { //
		covariance = BAD_COVARIANCE_MATRIX;
		whitening = BAD_WHITENING_MATRIX;
	}
}

void
covariancePositionRadius(const RawPoseCovarianceMatrix &raw_covariance, xrt_vec3 &radii, xrt_quat &Q_cam_cov)
{
	Eigen::Map<const PoseStateCovarianceMatrix> covariance(raw_covariance);

	const Eigen::Matrix3d &pos_covariance =
	    covariance.block<3, 3>(static_cast<int>(CovarianceIndex::PosX), static_cast<int>(CovarianceIndex::PosX));

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(pos_covariance);

	if (solver.info() != Eigen::Success) {
		// @todo handle failure better, if possible
		U_LOG_E("Failed to compute eigenvalues for position covariance matrix.");
		radii = {0.0f, 0.0f, 0.0f};
		Q_cam_cov = {0.0f, 0.0f, 0.0f, 1.0f};
		return;
	}

	const Eigen::Vector3d &eigenvalues = solver.eigenvalues();
	Eigen::Matrix3d eigenvectors = solver.eigenvectors();

	if (eigenvectors.determinant() < 0.0) {
		eigenvectors.col(2) *= -1.0;
	}

	Eigen::Quaterniond q(eigenvectors);
	map_quat(Q_cam_cov) = q.cast<float>();

	Eigen::Vector3d half_sizes = 3.0 * eigenvalues.cwiseMax(0.0).cwiseSqrt();
	map_vec3(radii) = half_sizes.cast<float>();
}

}; // namespace xrt::tracking::constellation::optimizer
