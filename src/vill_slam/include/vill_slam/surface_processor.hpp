/**
 * @file surface_processor.hpp
 * @brief Surface processing for pipe/corridor wall detection
 */

#ifndef VILL_SLAM_SURFACE_PROCESSOR_HPP
#define VILL_SLAM_SURFACE_PROCESSOR_HPP

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <memory>

namespace vill_slam {

// Local structures (avoid including GTSAM-dependent headers)

/**
 * @brief Represents a measured pipe cross-section
 */
struct PipeCrossSectionData {
    Eigen::Vector3d center;
    double radius;
    double radius_variance;
    Eigen::Vector3d axis_direction;
    double eccentricity;
    double confidence;
    std::vector<Eigen::Vector3d> points;

    PipeCrossSectionData()
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
struct CylinderFitData {
    Eigen::Vector3d axis;
    Eigen::Vector3d center;
    double radius;
    double fit_error;
    bool is_valid;

    CylinderFitData()
        : axis(Eigen::Vector3d::UnitZ()),
          center(Eigen::Vector3d::Zero()),
          radius(0.0),
          fit_error(0.0),
          is_valid(false) {}
};

/**
 * @brief Detected wall plane
 */
struct WallPlaneData {
    Eigen::Vector3d normal;
    double distance;
    double confidence;
    std::vector<Eigen::Vector3d> inlier_points;

    WallPlaneData()
        : normal(Eigen::Vector3d::UnitX()),
          distance(0.0),
          confidence(0.0) {}
};

enum class EnvironmentMode {
    UNKNOWN = 0,
    CORRIDOR = 1,
    PIPE = 2,
    AUTO = 3
};

/**
 * @brief Surface processing and environment detection
 */
struct SurfaceProcessorConfig {
    // Detection thresholds
    double cylinder_fit_threshold;
    double plane_fit_threshold;
    int min_points;

    // Cylinder detection
    double min_radius;
    double max_radius;
    double curvature_threshold;

    // Environment classification
    double pipe_confidence_threshold;
    double corridor_confidence_threshold;

    SurfaceProcessorConfig()
        : cylinder_fit_threshold(0.05),
          plane_fit_threshold(0.03),
          min_points(50),
          min_radius(0.1),
          max_radius(3.0),
          curvature_threshold(0.1),
          pipe_confidence_threshold(0.7),
          corridor_confidence_threshold(0.7) {}
};

class SurfaceProcessor {
public:
    using Config = SurfaceProcessorConfig;

    SurfaceProcessor(const Config& config = Config());

    /**
     * @brief Process laser line points and detect surface type
     */
    void processLaserLine(const std::vector<Eigen::Vector3d>& points);

    /**
     * @brief Process point cloud for surface detection
     */
    void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief Detect environment type (pipe or corridor)
     */
    EnvironmentMode detectEnvironment() const;

    /**
     * @brief Get detected pipe cross-section (if in pipe mode)
     */
    PipeCrossSectionData getPipeCrossSection() const { return pipe_section_; }

    /**
     * @brief Get detected wall planes (if in corridor mode)
     */
    std::vector<WallPlaneData> getWallPlanes() const { return wall_planes_; }

    /**
     * @brief Get confidence scores
     */
    double getPipeConfidence() const { return pipe_confidence_; }
    double getCorridorConfidence() const { return corridor_confidence_; }

    /**
     * @brief Force environment mode
     */
    void setEnvironmentMode(EnvironmentMode mode) { forced_mode_ = mode; }

private:
    Config config_;

    // Detection results
    PipeCrossSectionData pipe_section_;
    std::vector<WallPlaneData> wall_planes_;
    double pipe_confidence_ = 0.0;
    double corridor_confidence_ = 0.0;
    EnvironmentMode forced_mode_ = EnvironmentMode::AUTO;

    // Internal methods
    CylinderFitData fitCylinderInternal(const std::vector<Eigen::Vector3d>& points);
    std::vector<WallPlaneData> fitPlanesInternal(const std::vector<Eigen::Vector3d>& points);
    double computeCurvature(const std::vector<Eigen::Vector3d>& points);
};

}  // namespace vill_slam

#endif  // VILL_SLAM_SURFACE_PROCESSOR_HPP
