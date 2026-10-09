// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Generic math helpers for constellation optimizer code.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "math/m_eigen_interop.hpp"
#include "math/m_quatexpmap.hpp"
#include "math/m_quatexpmap_bigceres.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/jet.h>
#include <ceres/product_manifold.h>
#include <ceres/autodiff_manifold.h>

#include "pose_optimize.hpp"


namespace xrt::tracking::constellation::optimizer {

using namespace xrt::auxiliary::math;

// Covariance lives in the manifold's tangent space, which is one dimension smaller than the ambient
// parameter block: the quaternion contributes 3 tangent dimensions, not 4.
typedef Eigen::Matrix<double, kPoseCovarianceSize, kPoseCovarianceSize> PoseStateCovarianceMatrix;

//! Returns an Isometry3f from an @ref xrt_pose.
inline Eigen::Isometry3f
isometryFromPose(const xrt_pose &p)
{
	return {Eigen::Translation3f{map_vec3(p.position)} * map_quat(p.orientation)};
}

/*!
 * Right-multiplying quaternion manifold, matches our local convention.
 */
struct QuaternionManifoldFunctor
{
	template <typename T>
	bool
	Plus(const T *x, const T *delta, T *x_plus_delta) const
	{
		const Eigen::Map<const Eigen::Quaternion<T>> q(x);
		const Eigen::Map<const Eigen::Vector3<T>> w(delta);
		Eigen::Map<Eigen::Quaternion<T>> result(x_plus_delta);

		result = q * quat_exp_so3(w);
		return true;
	}

	template <typename T>
	bool
	Minus(const T *y, const T *x, T *y_minus_x) const
	{
		const Eigen::Map<const Eigen::Quaternion<T>> q_y(y);
		const Eigen::Map<const Eigen::Quaternion<T>> q_x(x);
		Eigen::Map<Eigen::Vector3<T>> result(y_minus_x);

		result = quat_ln_so3(Eigen::Quaternion<T>(q_x.conjugate() * q_y));
		return true;
	}
};

// Pose manifold, (tX, tY, tZ), (qX, qY, qZ, qW)
typedef ceres::ProductManifold<ceres::EuclideanManifold<3>, ceres::AutoDiffManifold<QuaternionManifoldFunctor, 4, 3>>
    PoseManifold;

// Pose plus linear velocity, (tX, tY, tZ), (qX, qY, qZ, qW), (vX, vY, vZ). 10 ambient, 9 tangent.
typedef ceres::ProductManifold<ceres::EuclideanManifold<3>,
                               ceres::AutoDiffManifold<QuaternionManifoldFunctor, 4, 3>,
                               ceres::EuclideanManifold<3>>
    PoseWithVelocityManifold;

}; // namespace xrt::tracking::constellation::optimizer
