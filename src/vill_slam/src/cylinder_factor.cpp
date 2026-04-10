/**
 * @file cylinder_factor.cpp
 * @brief Implementation of Cylinder Factor for pipe SLAM
 */

#include "vill_slam/cylinder_factor.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace vill_slam {

CylinderFitResult fitCylinder(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Vector3d& initial_axis)
{
    CylinderFitResult result;

    if (points.size() < 5) {
        result.is_valid = false;
        return result;
    }

    // Compute centroid
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        centroid += p;
    }
    centroid /= static_cast<double>(points.size());

    // Center the points
    std::vector<Eigen::Vector3d> centered(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        centered[i] = points[i] - centroid;
    }

    // Use PCA to find the principal axis (cylinder axis)
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& p : centered) {
        covariance += p * p.transpose();
    }
    covariance /= static_cast<double>(points.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();

    // The axis corresponds to the eigenvector with smallest eigenvalue
    // (points are distributed around the axis, not along it)
    int min_idx = 0;
    double min_val = eigenvalues(0);
    for (int i = 1; i < 3; ++i) {
        if (eigenvalues(i) < min_val) {
            min_val = eigenvalues(i);
            min_idx = i;
        }
    }

    Eigen::Vector3d axis = eigenvectors.col(min_idx).normalized();

    // Ensure axis points in same direction as initial guess
    if (axis.dot(initial_axis) < 0) {
        axis = -axis;
    }

    // Project points onto plane perpendicular to axis and fit circle
    std::vector<Eigen::Vector2d> projected;
    Eigen::Vector3d u, v;

    // Create orthonormal basis in the plane
    if (std::abs(axis.dot(Eigen::Vector3d::UnitX())) < 0.9) {
        u = axis.cross(Eigen::Vector3d::UnitX()).normalized();
    } else {
        u = axis.cross(Eigen::Vector3d::UnitY()).normalized();
    }
    v = axis.cross(u).normalized();

    for (const auto& p : centered) {
        // Project onto plane
        Eigen::Vector3d proj = p - axis * axis.dot(p);
        projected.emplace_back(proj.dot(u), proj.dot(v));
    }

    // Fit circle to 2D projected points using algebraic method
    // Minimize sum of (x^2 + y^2 - 2*cx*x - 2*cy*y + cx^2 + cy^2 - r^2)^2
    Eigen::MatrixXd A(projected.size(), 3);
    Eigen::VectorXd b(projected.size());

    for (size_t i = 0; i < projected.size(); ++i) {
        A(i, 0) = projected[i].x();
        A(i, 1) = projected[i].y();
        A(i, 2) = 1.0;
        b(i) = projected[i].squaredNorm();
    }

    // Solve least squares: A * [2*cx, 2*cy, r^2 - cx^2 - cy^2]^T = b
    Eigen::Vector3d params = A.colPivHouseholderQr().solve(b);
    double cx = params(0) / 2.0;
    double cy = params(1) / 2.0;
    double r_squared = params(2) + cx * cx + cy * cy;

    if (r_squared < 0) {
        result.is_valid = false;
        return result;
    }

    double radius = std::sqrt(r_squared);

    // Convert center back to 3D
    Eigen::Vector3d center_offset = cx * u + cy * v;
    Eigen::Vector3d center = centroid + center_offset;

    // Compute fitting error
    double total_error = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        double dist_to_center = std::sqrt(
            std::pow(projected[i].x() - cx, 2) +
            std::pow(projected[i].y() - cy, 2));
        double error = std::abs(dist_to_center - radius);
        total_error += error * error;
    }
    double rms_error = std::sqrt(total_error / projected.size());

    result.axis = axis;
    result.center = center;
    result.radius = radius;
    result.fit_error = rms_error;
    result.is_valid = true;

    return result;
}

Eigen::Vector3d computeCylinderError(
    const PipeCrossSection& measured_section,
    double estimated_radius,
    const Eigen::Isometry3d& pose)
{
    Eigen::Vector3d error = Eigen::Vector3d::Zero();

    // Transform measured points to world frame
    std::vector<Eigen::Vector3d> world_points;
    world_points.reserve(measured_section.points.size());
    for (const auto& p : measured_section.points) {
        world_points.push_back(pose * p);
    }

    // Fit cylinder to world-frame points
    CylinderFitResult fit = fitCylinder(world_points, pose.rotation() * measured_section.axis_direction);

    if (!fit.is_valid) {
        // Return zero error if fitting fails
        return error;
    }

    // Error 1: Radius difference
    error(0) = measured_section.radius - estimated_radius;

    // Error 2: Axis alignment (cross product magnitude)
    // This measures angular deviation between measured and estimated axis
    Eigen::Vector3d axis_cross = fit.axis.cross(measured_section.axis_direction);
    error(1) = axis_cross.norm();

    // Error 3: Distance from pipe center
    // Robot should stay near the center of the pipe
    Eigen::Vector3d robot_pos = pose.translation();
    Eigen::Vector3d to_robot = robot_pos - fit.center;

    // Project onto plane perpendicular to axis
    Eigen::Vector3d perpendicular = to_robot - fit.axis * fit.axis.dot(to_robot);
    error(2) = perpendicular.norm();

    return error;
}

#ifdef USE_GTSAM

CylinderFactor::CylinderFactor(
    gtsam::Key pose_key,
    gtsam::Key radius_key,
    const PipeCrossSection& measured,
    const gtsam::SharedNoiseModel& noise_model)
    : Base(noise_model, pose_key, radius_key),
      measured_(measured) {}

gtsam::Vector CylinderFactor::evaluateError(
    const gtsam::Pose3& pose,
    const gtsam::Vector1& radius,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const
{
    // Convert GTSAM pose to Eigen
    Eigen::Isometry3d eigen_pose = Eigen::Isometry3d::Identity();
    eigen_pose.linear() = pose.rotation().matrix();
    eigen_pose.translation() = pose.translation();

    // Compute error
    Eigen::Vector3d error = computeCylinderError(measured_, radius(0), eigen_pose);

    // Numerical Jacobians (for simplicity - can be replaced with analytical)
    if (H1) {
        const double eps = 1e-5;
        *H1 = gtsam::Matrix::Zero(3, 6);

        // Jacobian w.r.t. rotation (3 parameters)
        for (int i = 0; i < 3; ++i) {
            gtsam::Vector3 delta = gtsam::Vector3::Zero();
            delta(i) = eps;
            gtsam::Pose3 pose_plus = pose * gtsam::Pose3::Expmap(
                (gtsam::Vector6() << delta, gtsam::Vector3::Zero()).finished());

            Eigen::Isometry3d eigen_pose_plus = Eigen::Isometry3d::Identity();
            eigen_pose_plus.linear() = pose_plus.rotation().matrix();
            eigen_pose_plus.translation() = pose_plus.translation();

            Eigen::Vector3d error_plus = computeCylinderError(measured_, radius(0), eigen_pose_plus);
            H1->col(i) = (error_plus - error) / eps;
        }

        // Jacobian w.r.t. translation (3 parameters)
        for (int i = 0; i < 3; ++i) {
            gtsam::Vector3 delta = gtsam::Vector3::Zero();
            delta(i) = eps;
            gtsam::Pose3 pose_plus = pose * gtsam::Pose3::Expmap(
                (gtsam::Vector6() << gtsam::Vector3::Zero(), delta).finished());

            Eigen::Isometry3d eigen_pose_plus = Eigen::Isometry3d::Identity();
            eigen_pose_plus.linear() = pose_plus.rotation().matrix();
            eigen_pose_plus.translation() = pose_plus.translation();

            Eigen::Vector3d error_plus = computeCylinderError(measured_, radius(0), eigen_pose_plus);
            H1->col(3 + i) = (error_plus - error) / eps;
        }
    }

    if (H2) {
        // Jacobian w.r.t. radius
        *H2 = gtsam::Matrix::Zero(3, 1);
        (*H2)(0, 0) = -1.0;  // d(r_meas - r_est)/d(r_est) = -1
    }

    return error;
}

gtsam::NonlinearFactor::shared_ptr createCylinderFactor(
    gtsam::Key pose_key,
    gtsam::Key radius_key,
    const PipeCrossSection& measured,
    double weight)
{
    // Create noise model based on measurement confidence
    double sigma_r = 0.01 / (measured.confidence + 0.01);  // Radius error std
    double sigma_a = 0.1 / (measured.confidence + 0.01);   // Axis error std
    double sigma_c = 0.05 / (measured.confidence + 0.01);  // Center offset std

    gtsam::Vector3 sigmas;
    sigmas << sigma_r * weight, sigma_a * weight, sigma_c * weight;
    auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    return boost::make_shared<CylinderFactor>(pose_key, radius_key, measured, noise_model);
}

#endif  // USE_GTSAM

// Standalone constraint implementation
CylinderConstraint::CylinderConstraint(const PipeCrossSection& measured, double weight)
    : measured_(measured), weight_(weight) {}

Eigen::Vector3d CylinderConstraint::computeResidual(
    const Eigen::Matrix<double, 7, 1>& pose,
    double radius) const
{
    // Extract quaternion and translation from pose vector
    Eigen::Quaterniond q(pose(0), pose(1), pose(2), pose(3));
    Eigen::Vector3d t(pose(4), pose(5), pose(6));

    Eigen::Isometry3d eigen_pose = Eigen::Isometry3d::Identity();
    eigen_pose.linear() = q.toRotationMatrix();
    eigen_pose.translation() = t;

    return weight_ * computeCylinderError(measured_, radius, eigen_pose);
}

Eigen::Vector3d CylinderConstraint::computeResidualWithJacobian(
    const Eigen::Matrix<double, 7, 1>& pose,
    double radius,
    Eigen::Matrix<double, 3, 7>& J_pose,
    Eigen::Matrix<double, 3, 1>& J_radius) const
{
    const double eps = 1e-7;

    // Compute base residual
    Eigen::Vector3d residual = computeResidual(pose, radius);

    // Numerical Jacobian w.r.t. pose
    J_pose = Eigen::Matrix<double, 3, 7>::Zero();
    for (int i = 0; i < 7; ++i) {
        Eigen::Matrix<double, 7, 1> pose_plus = pose;
        pose_plus(i) += eps;

        // Renormalize quaternion if perturbed
        if (i < 4) {
            Eigen::Quaterniond q(pose_plus(0), pose_plus(1), pose_plus(2), pose_plus(3));
            q.normalize();
            pose_plus(0) = q.w();
            pose_plus(1) = q.x();
            pose_plus(2) = q.y();
            pose_plus(3) = q.z();
        }

        Eigen::Vector3d residual_plus = computeResidual(pose_plus, radius);
        J_pose.col(i) = (residual_plus - residual) / eps;
    }

    // Jacobian w.r.t. radius
    Eigen::Vector3d residual_plus = computeResidual(pose, radius + eps);
    J_radius = (residual_plus - residual) / eps;

    return residual;
}

}  // namespace vill_slam
