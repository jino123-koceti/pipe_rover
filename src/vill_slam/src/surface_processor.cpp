/**
 * @file surface_processor.cpp
 * @brief Implementation of surface processing
 */

#include "vill_slam/surface_processor.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace vill_slam {

SurfaceProcessor::SurfaceProcessor(const Config& config)
    : config_(config) {}

void SurfaceProcessor::processLaserLine(const std::vector<Eigen::Vector3d>& points)
{
    if (static_cast<int>(points.size()) < config_.min_points) {
        pipe_confidence_ = 0.0;
        corridor_confidence_ = 0.0;
        return;
    }

    // Try fitting both cylinder and planes
    CylinderFitData fit = fitCylinderInternal(points);
    wall_planes_ = fitPlanesInternal(points);

    // Update pipe section if fit is valid
    if (fit.is_valid &&
        fit.radius >= config_.min_radius &&
        fit.radius <= config_.max_radius &&
        fit.fit_error < config_.cylinder_fit_threshold) {

        pipe_section_.center = fit.center;
        pipe_section_.radius = fit.radius;
        pipe_section_.axis_direction = fit.axis;
        pipe_section_.points = points;
        pipe_section_.eccentricity = std::min(1.0, fit.fit_error / config_.cylinder_fit_threshold);

        double count_factor = std::min(1.0, static_cast<double>(points.size()) / 200.0);
        double fit_factor = 1.0 - (fit.fit_error / config_.cylinder_fit_threshold);
        pipe_section_.confidence = count_factor * fit_factor;
        pipe_section_.radius_variance = fit.fit_error * fit.fit_error;
    } else {
        pipe_section_ = PipeCrossSectionData();
    }

    // Update confidence based on fit quality
    if (pipe_section_.confidence > 0 && pipe_section_.eccentricity < 0.3) {
        pipe_confidence_ = pipe_section_.confidence * (1.0 - pipe_section_.eccentricity);
    } else {
        pipe_confidence_ = 0.0;
    }

    // Corridor confidence based on parallel walls detection
    if (wall_planes_.size() >= 2) {
        double avg_confidence = 0.0;
        for (const auto& wall : wall_planes_) {
            avg_confidence += wall.confidence;
        }
        avg_confidence /= static_cast<double>(wall_planes_.size());

        // Check if walls are parallel
        double dot = std::abs(wall_planes_[0].normal.dot(wall_planes_[1].normal));
        if (dot > 0.9) {  // Nearly parallel
            corridor_confidence_ = avg_confidence * dot;
        }
    }
}

void SurfaceProcessor::processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (!cloud || static_cast<int>(cloud->points.size()) < config_.min_points) {
        return;
    }

    // Convert to Eigen points
    std::vector<Eigen::Vector3d> points;
    points.reserve(cloud->points.size());
    for (const auto& pt : cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            points.emplace_back(pt.x, pt.y, pt.z);
        }
    }

    processLaserLine(points);
}

EnvironmentMode SurfaceProcessor::detectEnvironment() const
{
    if (forced_mode_ != EnvironmentMode::AUTO) {
        return forced_mode_;
    }

    if (pipe_confidence_ > config_.pipe_confidence_threshold &&
        pipe_confidence_ > corridor_confidence_) {
        return EnvironmentMode::PIPE;
    }

    if (corridor_confidence_ > config_.corridor_confidence_threshold) {
        return EnvironmentMode::CORRIDOR;
    }

    return EnvironmentMode::UNKNOWN;
}

CylinderFitData SurfaceProcessor::fitCylinderInternal(const std::vector<Eigen::Vector3d>& points)
{
    CylinderFitData result;

    if (points.size() < 10) {
        return result;
    }

    // Compute centroid
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        centroid += p;
    }
    centroid /= static_cast<double>(points.size());

    // PCA for axis estimation
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& p : points) {
        Eigen::Vector3d centered = p - centroid;
        covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(points.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();

    // Axis is the direction of least variance (smallest eigenvalue)
    result.axis = eigenvectors.col(0).normalized();
    result.center = centroid;

    // Estimate radius: average distance from axis
    double total_radius = 0.0;
    double total_error = 0.0;

    for (const auto& p : points) {
        Eigen::Vector3d to_point = p - centroid;
        Eigen::Vector3d on_axis = to_point.dot(result.axis) * result.axis;
        Eigen::Vector3d perpendicular = to_point - on_axis;
        double dist = perpendicular.norm();
        total_radius += dist;
    }

    result.radius = total_radius / static_cast<double>(points.size());

    // Compute fit error
    for (const auto& p : points) {
        Eigen::Vector3d to_point = p - centroid;
        Eigen::Vector3d on_axis = to_point.dot(result.axis) * result.axis;
        Eigen::Vector3d perpendicular = to_point - on_axis;
        double dist = perpendicular.norm();
        double error = std::abs(dist - result.radius);
        total_error += error * error;
    }

    result.fit_error = std::sqrt(total_error / static_cast<double>(points.size()));
    result.is_valid = (result.radius > 0.05 && result.fit_error < 0.5);

    return result;
}

std::vector<WallPlaneData> SurfaceProcessor::fitPlanesInternal(const std::vector<Eigen::Vector3d>& points)
{
    std::vector<WallPlaneData> planes;

    if (points.size() < 20) {
        return planes;
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : points) {
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(p.x());
        pt.y = static_cast<float>(p.y());
        pt.z = static_cast<float>(p.z());
        cloud->points.push_back(pt);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    // RANSAC plane fitting
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(config_.plane_fit_threshold);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Try to find up to 2 planes
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud = cloud;

    for (int i = 0; i < 2 && remaining_cloud->points.size() > 20; ++i) {
        seg.setInputCloud(remaining_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() < 10) {
            break;
        }

        WallPlaneData wall;
        wall.normal = Eigen::Vector3d(
            coefficients->values[0],
            coefficients->values[1],
            coefficients->values[2]);
        wall.distance = coefficients->values[3];
        wall.confidence = static_cast<double>(inliers->indices.size()) /
                          static_cast<double>(remaining_cloud->points.size());

        for (int idx : inliers->indices) {
            const auto& pt = remaining_cloud->points[idx];
            wall.inlier_points.emplace_back(pt.x, pt.y, pt.z);
        }

        planes.push_back(wall);

        // Remove inliers for next iteration
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t j = 0; j < remaining_cloud->points.size(); ++j) {
            if (std::find(inliers->indices.begin(), inliers->indices.end(), j) ==
                inliers->indices.end()) {
                new_cloud->points.push_back(remaining_cloud->points[j]);
            }
        }
        new_cloud->width = new_cloud->points.size();
        new_cloud->height = 1;
        remaining_cloud = new_cloud;
    }

    return planes;
}

double SurfaceProcessor::computeCurvature(const std::vector<Eigen::Vector3d>& points)
{
    if (points.size() < 10) return 0.0;

    // Compute local curvature using PCA
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        centroid += p;
    }
    centroid /= static_cast<double>(points.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& p : points) {
        Eigen::Vector3d centered = p - centroid;
        covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(points.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    Eigen::Vector3d eigenvalues = solver.eigenvalues();

    // Curvature measure: ratio of smallest to sum of eigenvalues
    double total = eigenvalues.sum();
    if (total < 1e-6) return 0.0;

    return eigenvalues(0) / total;
}

}  // namespace vill_slam
