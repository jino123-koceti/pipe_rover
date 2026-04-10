/**
 * @file plane_factor.hpp
 * @brief Plane Factor for corridor environment SLAM
 *
 * Implements a factor graph constraint for planar walls
 * Used for corridor testing when pipe environment is not available
 */

#ifndef VILL_SLAM_PLANE_FACTOR_HPP
#define VILL_SLAM_PLANE_FACTOR_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>

#ifdef USE_GTSAM
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#endif

namespace vill_slam {

/**
 * @brief Represents a measured wall plane
 */
struct WallPlane {
    Eigen::Vector4d coefficients;     // Plane equation: ax + by + cz + d = 0
    Eigen::Vector3d normal;           // Normal vector (a, b, c)
    Eigen::Vector3d centroid;         // Center point of measured region
    double distance;                  // Distance from origin
    double confidence;                // Detection confidence
    double fit_error;                 // Fitting RMS error
    std::vector<Eigen::Vector3d> points;  // Points on the plane

    WallPlane()
        : coefficients(Eigen::Vector4d::Zero()),
          normal(Eigen::Vector3d::UnitZ()),
          centroid(Eigen::Vector3d::Zero()),
          distance(0.0),
          confidence(0.0),
          fit_error(0.0) {}
};

/**
 * @brief Plane fitting result
 */
struct PlaneFitResult {
    Eigen::Vector4d coefficients;     // [a, b, c, d] where ax+by+cz+d=0
    Eigen::Vector3d normal;           // Unit normal
    double distance;                  // Distance from origin
    double fit_error;                 // RMS fitting error
    bool is_valid;

    PlaneFitResult()
        : coefficients(Eigen::Vector4d::Zero()),
          normal(Eigen::Vector3d::UnitZ()),
          distance(0.0),
          fit_error(0.0),
          is_valid(false) {}
};

/**
 * @brief Fits a plane to a set of 3D points using SVD
 *
 * @param points Input 3D points
 * @return PlaneFitResult containing fitted parameters
 */
PlaneFitResult fitPlane(const std::vector<Eigen::Vector3d>& points);

/**
 * @brief RANSAC-based plane fitting for noisy data
 *
 * @param points Input 3D points
 * @param distance_threshold Maximum distance for inliers
 * @param max_iterations Maximum RANSAC iterations
 * @return PlaneFitResult from best model
 */
PlaneFitResult fitPlaneRANSAC(
    const std::vector<Eigen::Vector3d>& points,
    double distance_threshold = 0.02,
    int max_iterations = 100);

/**
 * @brief Computes error between measured and estimated plane
 *
 * @param measured_plane Measured wall plane
 * @param estimated_plane Estimated plane parameters [a, b, c, d]
 * @param pose Current robot pose
 * @return Error vector [normal_alignment, distance_error, lateral_offset]
 */
Eigen::Vector3d computePlaneError(
    const WallPlane& measured_plane,
    const Eigen::Vector4d& estimated_plane,
    const Eigen::Isometry3d& pose);

#ifdef USE_GTSAM

/**
 * @brief GTSAM Factor for plane constraint
 *
 * This factor constrains the robot pose to be consistent with
 * observed planar wall geometry. Useful for corridor environments.
 *
 * State variables:
 *   - X(k): Robot pose (Pose3)
 *   - P(0): Plane coefficients (Vector4)
 *
 * Error function:
 *   e = [w_n * (1 - |n_measured · n_estimated|),
 *        w_d * (d_measured - d_estimated),
 *        w_l * lateral_offset]
 */
class PlaneFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector4> {
public:
    using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector4>;

    /**
     * @brief Constructor
     *
     * @param pose_key Key for robot pose X(k)
     * @param plane_key Key for plane parameters P(0)
     * @param measured Measured wall plane
     * @param noise_model Noise model (3-dimensional)
     */
    PlaneFactor(
        gtsam::Key pose_key,
        gtsam::Key plane_key,
        const WallPlane& measured,
        const gtsam::SharedNoiseModel& noise_model);

    /**
     * @brief Evaluate error
     */
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        const gtsam::Vector4& plane,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::make_shared<PlaneFactor>(*this);
    }

private:
    WallPlane measured_;
};

/**
 * @brief Factory function to create PlaneFactor
 */
gtsam::NonlinearFactor::shared_ptr createPlaneFactor(
    gtsam::Key pose_key,
    gtsam::Key plane_key,
    const WallPlane& measured,
    double weight = 1.0);

#endif  // USE_GTSAM

/**
 * @brief Standalone plane constraint computation
 */
class PlaneConstraint {
public:
    PlaneConstraint(const WallPlane& measured, double weight = 1.0);

    /**
     * @brief Compute residual
     *
     * @param pose Robot pose as 7-vector [qw, qx, qy, qz, tx, ty, tz]
     * @param plane Plane parameters [a, b, c, d]
     * @return 3-dimensional residual
     */
    Eigen::Vector3d computeResidual(
        const Eigen::Matrix<double, 7, 1>& pose,
        const Eigen::Vector4d& plane) const;

    const WallPlane& getMeasurement() const { return measured_; }
    double getWeight() const { return weight_; }

private:
    WallPlane measured_;
    double weight_;
};

/**
 * @brief Detects corridor walls from laser line data
 *
 * Identifies parallel walls typical of corridor environments
 *
 * @param points 3D points from line laser
 * @param min_wall_distance Minimum distance between walls
 * @return Vector of detected wall planes (usually 2 for corridors)
 */
std::vector<WallPlane> detectCorridorWalls(
    const std::vector<Eigen::Vector3d>& points,
    double min_wall_distance = 1.0);

}  // namespace vill_slam

#endif  // VILL_SLAM_PLANE_FACTOR_HPP
