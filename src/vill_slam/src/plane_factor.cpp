/**
 * @file plane_factor.cpp
 * @brief Implementation of Plane Factor for corridor SLAM
 */

#include "vill_slam/plane_factor.hpp"
#include <cmath>
#include <algorithm>
#include <random>

namespace vill_slam {

PlaneFitResult fitPlane(const std::vector<Eigen::Vector3d>& points)
{
    PlaneFitResult result;

    if (points.size() < 3) {
        result.is_valid = false;
        return result;
    }

    // Compute centroid
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        centroid += p;
    }
    centroid /= static_cast<double>(points.size());

    // Build matrix of centered points
    Eigen::MatrixXd A(points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        A.row(i) = (points[i] - centroid).transpose();
    }

    // SVD to find normal (smallest singular value)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
    Eigen::Vector3d normal = svd.matrixV().col(2);

    // Ensure normal points in consistent direction (e.g., positive z)
    if (normal.z() < 0) {
        normal = -normal;
    }

    // Distance from origin
    double d = -normal.dot(centroid);

    // Plane coefficients
    result.coefficients << normal.x(), normal.y(), normal.z(), d;
    result.normal = normal;
    result.distance = std::abs(d);

    // Compute fitting error
    double total_error = 0.0;
    for (const auto& p : points) {
        double dist = std::abs(normal.dot(p) + d);
        total_error += dist * dist;
    }
    result.fit_error = std::sqrt(total_error / points.size());
    result.is_valid = true;

    return result;
}

PlaneFitResult fitPlaneRANSAC(
    const std::vector<Eigen::Vector3d>& points,
    double distance_threshold,
    int max_iterations)
{
    PlaneFitResult best_result;
    best_result.is_valid = false;
    int best_inlier_count = 0;

    if (points.size() < 3) {
        return best_result;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Sample 3 random points
        size_t i1 = dist(gen);
        size_t i2 = dist(gen);
        size_t i3 = dist(gen);

        // Ensure distinct points
        while (i2 == i1) i2 = dist(gen);
        while (i3 == i1 || i3 == i2) i3 = dist(gen);

        // Fit plane to 3 points
        Eigen::Vector3d v1 = points[i2] - points[i1];
        Eigen::Vector3d v2 = points[i3] - points[i1];
        Eigen::Vector3d normal = v1.cross(v2);

        if (normal.norm() < 1e-6) continue;

        normal.normalize();
        double d = -normal.dot(points[i1]);

        // Count inliers
        int inlier_count = 0;
        std::vector<Eigen::Vector3d> inliers;
        for (const auto& p : points) {
            double dist = std::abs(normal.dot(p) + d);
            if (dist < distance_threshold) {
                ++inlier_count;
                inliers.push_back(p);
            }
        }

        // Update best if more inliers
        if (inlier_count > best_inlier_count && inliers.size() >= 3) {
            best_inlier_count = inlier_count;

            // Refit using all inliers
            PlaneFitResult refined = fitPlane(inliers);
            if (refined.is_valid) {
                best_result = refined;
            }
        }
    }

    return best_result;
}

Eigen::Vector3d computePlaneError(
    const WallPlane& measured_plane,
    const Eigen::Vector4d& estimated_plane,
    const Eigen::Isometry3d& pose)
{
    Eigen::Vector3d error = Eigen::Vector3d::Zero();

    // Extract normals
    Eigen::Vector3d measured_normal = measured_plane.normal.normalized();
    Eigen::Vector3d estimated_normal = estimated_plane.head<3>().normalized();

    // Transform measured normal to world frame
    Eigen::Vector3d world_normal = pose.rotation() * measured_normal;

    // Error 1: Normal alignment (1 - |dot product|)
    double dot = world_normal.dot(estimated_normal);
    error(0) = 1.0 - std::abs(dot);

    // Error 2: Distance difference
    double measured_dist = measured_plane.distance;
    double estimated_dist = std::abs(estimated_plane(3)) /
                           estimated_plane.head<3>().norm();
    error(1) = measured_dist - estimated_dist;

    // Error 3: Lateral offset from wall
    // Distance from robot position to estimated plane
    Eigen::Vector3d robot_pos = pose.translation();
    double dist_to_plane = std::abs(
        estimated_normal.dot(robot_pos) + estimated_plane(3) /
        estimated_plane.head<3>().norm());
    error(2) = dist_to_plane;

    return error;
}

#ifdef USE_GTSAM

PlaneFactor::PlaneFactor(
    gtsam::Key pose_key,
    gtsam::Key plane_key,
    const WallPlane& measured,
    const gtsam::SharedNoiseModel& noise_model)
    : Base(noise_model, pose_key, plane_key),
      measured_(measured) {}

gtsam::Vector PlaneFactor::evaluateError(
    const gtsam::Pose3& pose,
    const gtsam::Vector4& plane,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const
{
    // Convert GTSAM pose to Eigen
    Eigen::Isometry3d eigen_pose = Eigen::Isometry3d::Identity();
    eigen_pose.linear() = pose.rotation().matrix();
    eigen_pose.translation() = pose.translation();

    // Compute error
    Eigen::Vector3d error = computePlaneError(measured_, plane, eigen_pose);

    // Numerical Jacobians
    if (H1) {
        const double eps = 1e-5;
        *H1 = gtsam::Matrix::Zero(3, 6);

        for (int i = 0; i < 6; ++i) {
            gtsam::Vector6 delta = gtsam::Vector6::Zero();
            delta(i) = eps;
            gtsam::Pose3 pose_plus = pose * gtsam::Pose3::Expmap(delta);

            Eigen::Isometry3d eigen_pose_plus = Eigen::Isometry3d::Identity();
            eigen_pose_plus.linear() = pose_plus.rotation().matrix();
            eigen_pose_plus.translation() = pose_plus.translation();

            Eigen::Vector3d error_plus = computePlaneError(measured_, plane, eigen_pose_plus);
            H1->col(i) = (error_plus - error) / eps;
        }
    }

    if (H2) {
        const double eps = 1e-5;
        *H2 = gtsam::Matrix::Zero(3, 4);

        for (int i = 0; i < 4; ++i) {
            gtsam::Vector4 plane_plus = plane;
            plane_plus(i) += eps;
            Eigen::Vector3d error_plus = computePlaneError(measured_, plane_plus, eigen_pose);
            H2->col(i) = (error_plus - error) / eps;
        }
    }

    return error;
}

gtsam::NonlinearFactor::shared_ptr createPlaneFactor(
    gtsam::Key pose_key,
    gtsam::Key plane_key,
    const WallPlane& measured,
    double weight)
{
    // Create noise model
    double sigma_n = 0.1 / (measured.confidence + 0.01);   // Normal alignment
    double sigma_d = 0.02 / (measured.confidence + 0.01);  // Distance
    double sigma_l = 0.05 / (measured.confidence + 0.01);  // Lateral offset

    gtsam::Vector3 sigmas;
    sigmas << sigma_n * weight, sigma_d * weight, sigma_l * weight;
    auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    return boost::make_shared<PlaneFactor>(pose_key, plane_key, measured, noise_model);
}

#endif  // USE_GTSAM

// Standalone constraint
PlaneConstraint::PlaneConstraint(const WallPlane& measured, double weight)
    : measured_(measured), weight_(weight) {}

Eigen::Vector3d PlaneConstraint::computeResidual(
    const Eigen::Matrix<double, 7, 1>& pose,
    const Eigen::Vector4d& plane) const
{
    Eigen::Quaterniond q(pose(0), pose(1), pose(2), pose(3));
    Eigen::Vector3d t(pose(4), pose(5), pose(6));

    Eigen::Isometry3d eigen_pose = Eigen::Isometry3d::Identity();
    eigen_pose.linear() = q.toRotationMatrix();
    eigen_pose.translation() = t;

    return weight_ * computePlaneError(measured_, plane, eigen_pose);
}

std::vector<WallPlane> detectCorridorWalls(
    const std::vector<Eigen::Vector3d>& points,
    double min_wall_distance)
{
    std::vector<WallPlane> walls;

    if (points.size() < 10) {
        return walls;
    }

    // Cluster points based on Y-coordinate (assuming corridor along X axis)
    std::vector<Eigen::Vector3d> left_points, right_points;

    // Find median Y to split points
    std::vector<double> y_values;
    for (const auto& p : points) {
        y_values.push_back(p.y());
    }
    std::sort(y_values.begin(), y_values.end());
    double median_y = y_values[y_values.size() / 2];

    for (const auto& p : points) {
        if (p.y() < median_y - min_wall_distance / 4) {
            left_points.push_back(p);
        } else if (p.y() > median_y + min_wall_distance / 4) {
            right_points.push_back(p);
        }
    }

    // Fit plane to each cluster
    if (left_points.size() >= 3) {
        PlaneFitResult fit = fitPlaneRANSAC(left_points, 0.02, 50);
        if (fit.is_valid) {
            WallPlane wall;
            wall.coefficients = fit.coefficients;
            wall.normal = fit.normal;
            wall.distance = fit.distance;
            wall.fit_error = fit.fit_error;
            wall.points = left_points;
            wall.confidence = std::min(1.0, static_cast<double>(left_points.size()) / 100.0);

            // Compute centroid
            wall.centroid = Eigen::Vector3d::Zero();
            for (const auto& p : left_points) {
                wall.centroid += p;
            }
            wall.centroid /= static_cast<double>(left_points.size());

            walls.push_back(wall);
        }
    }

    if (right_points.size() >= 3) {
        PlaneFitResult fit = fitPlaneRANSAC(right_points, 0.02, 50);
        if (fit.is_valid) {
            WallPlane wall;
            wall.coefficients = fit.coefficients;
            wall.normal = fit.normal;
            wall.distance = fit.distance;
            wall.fit_error = fit.fit_error;
            wall.points = right_points;
            wall.confidence = std::min(1.0, static_cast<double>(right_points.size()) / 100.0);

            wall.centroid = Eigen::Vector3d::Zero();
            for (const auto& p : right_points) {
                wall.centroid += p;
            }
            wall.centroid /= static_cast<double>(right_points.size());

            walls.push_back(wall);
        }
    }

    return walls;
}

}  // namespace vill_slam
