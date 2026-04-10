/**
 * @file cylinder_factor.hpp
 * @brief Cylinder Factor for pipe environment SLAM
 *
 * Implements a factor graph constraint that enforces cylindrical geometry
 * for pipe environments. This is the key innovation from CMU's VILL-SLAM.
 */

#ifndef VILL_SLAM_CYLINDER_FACTOR_HPP
#define VILL_SLAM_CYLINDER_FACTOR_HPP

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
 * @brief Represents a measured pipe cross-section
 */
struct PipeCrossSection {
    Eigen::Vector3d center;           // Center point in sensor frame
    double radius;                     // Measured radius
    double radius_variance;            // Uncertainty
    Eigen::Vector3d axis_direction;   // Pipe axis direction
    double eccentricity;              // Deviation from perfect circle
    double confidence;                // Detection confidence
    std::vector<Eigen::Vector3d> points;  // Cross-section points

    PipeCrossSection()
        : center(Eigen::Vector3d::Zero()),
          radius(0.0),
          radius_variance(0.01),
          axis_direction(Eigen::Vector3d::UnitZ()),
          eccentricity(0.0),
          confidence(0.0) {}
};

/**
 * @brief Cylinder fitting result
 */
struct CylinderFitResult {
    Eigen::Vector3d axis;          // Cylinder axis direction (unit vector)
    Eigen::Vector3d center;        // Point on the axis
    double radius;                 // Fitted radius
    double fit_error;              // RMS fitting error
    bool is_valid;                 // Whether fit succeeded

    CylinderFitResult()
        : axis(Eigen::Vector3d::UnitZ()),
          center(Eigen::Vector3d::Zero()),
          radius(0.0),
          fit_error(0.0),
          is_valid(false) {}
};

/**
 * @brief Fits a cylinder to a set of 3D points
 *
 * @param points Input 3D points (should be on cylinder surface)
 * @param initial_axis Initial guess for axis direction (optional)
 * @return CylinderFitResult containing fitted parameters
 */
CylinderFitResult fitCylinder(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Vector3d& initial_axis = Eigen::Vector3d::UnitZ());

/**
 * @brief Computes error between measured and estimated cylinder parameters
 *
 * @param measured_section Measured pipe cross-section
 * @param estimated_radius Estimated pipe radius from state
 * @param pose Current robot pose
 * @return Error vector [radius_error, axis_error, center_offset]
 */
Eigen::Vector3d computeCylinderError(
    const PipeCrossSection& measured_section,
    double estimated_radius,
    const Eigen::Isometry3d& pose);

#ifdef USE_GTSAM
/**
 * @brief GTSAM Factor for cylinder constraint
 *
 * This factor constrains the robot pose to be consistent with
 * observed cylindrical geometry.
 *
 * State variables:
 *   - X(k): Robot pose (Pose3)
 *   - R(0): Pipe radius (double, shared across all keyframes)
 *
 * Error function:
 *   e = [w_r * (r_measured - r_estimated),
 *        w_a * ||a_measured × a_estimated||,
 *        w_c * distance_from_axis]
 */
class CylinderFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector1> {
public:
    using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector1>;

    /**
     * @brief Constructor
     *
     * @param pose_key Key for robot pose X(k)
     * @param radius_key Key for pipe radius R(0)
     * @param measured Measured pipe cross-section
     * @param noise_model Noise model (3-dimensional)
     */
    CylinderFactor(
        gtsam::Key pose_key,
        gtsam::Key radius_key,
        const PipeCrossSection& measured,
        const gtsam::SharedNoiseModel& noise_model);

    /**
     * @brief Evaluate error
     *
     * @param pose Current robot pose
     * @param radius Current radius estimate (as Vector1)
     * @param H1 Optional Jacobian w.r.t. pose
     * @param H2 Optional Jacobian w.r.t. radius
     * @return Error vector
     */
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        const gtsam::Vector1& radius,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

    /// Clone for factor graph
    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::make_shared<CylinderFactor>(*this);
    }

private:
    PipeCrossSection measured_;
};

/**
 * @brief Factory function to create CylinderFactor
 */
gtsam::NonlinearFactor::shared_ptr createCylinderFactor(
    gtsam::Key pose_key,
    gtsam::Key radius_key,
    const PipeCrossSection& measured,
    double weight = 1.0);

#endif  // USE_GTSAM

/**
 * @brief Standalone cylinder constraint computation (when GTSAM not available)
 *
 * Can be used with other optimization libraries like Ceres or g2o
 */
class CylinderConstraint {
public:
    CylinderConstraint(const PipeCrossSection& measured, double weight = 1.0);

    /**
     * @brief Compute residual
     *
     * @param pose Robot pose as 7-vector [qw, qx, qy, qz, tx, ty, tz]
     * @param radius Pipe radius estimate
     * @return 3-dimensional residual
     */
    Eigen::Vector3d computeResidual(
        const Eigen::Matrix<double, 7, 1>& pose,
        double radius) const;

    /**
     * @brief Compute residual with Jacobians
     */
    Eigen::Vector3d computeResidualWithJacobian(
        const Eigen::Matrix<double, 7, 1>& pose,
        double radius,
        Eigen::Matrix<double, 3, 7>& J_pose,
        Eigen::Matrix<double, 3, 1>& J_radius) const;

    const PipeCrossSection& getMeasurement() const { return measured_; }
    double getWeight() const { return weight_; }

private:
    PipeCrossSection measured_;
    double weight_;
};

}  // namespace vill_slam

#endif  // VILL_SLAM_CYLINDER_FACTOR_HPP
