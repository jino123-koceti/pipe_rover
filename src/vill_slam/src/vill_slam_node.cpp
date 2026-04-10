/**
 * @file vill_slam_node.cpp
 * @brief Main VILL-SLAM ROS2 node with loop closure and pose graph optimization
 *
 * Integrates: COIN-LIO odometry + GTSAM iSAM2 pose graph + ICP loop closure
 *             + cylinder/plane geometric constraints
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/srv/trigger.hpp>

#include <vill_slam_msgs/msg/laser_line.hpp>
#include <vill_slam_msgs/msg/surface_section.hpp>
#include <vill_slam_msgs/msg/vill_slam_status.hpp>

#include "vill_slam/cylinder_factor.hpp"
#include "vill_slam/plane_factor.hpp"
#include "vill_slam/surface_processor.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef USE_GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#endif

#include <Eigen/Dense>
#include <mutex>
#include <deque>
#include <chrono>

namespace vill_slam {

#ifdef USE_GTSAM
using gtsam::symbol_shorthand::X;  // Pose3 (keyframe poses)
using gtsam::symbol_shorthand::R;  // Vector1 (pipe radius)
using gtsam::symbol_shorthand::P;  // Vector4 (plane params)
#endif

// Keyframe structure for loop closure
struct Keyframe {
    uint64_t id;
    rclcpp::Time stamp;
    Eigen::Isometry3d odom_pose;       // Raw odometry pose from COIN-LIO
    Eigen::Isometry3d optimized_pose;  // Pose after graph optimization
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // Downsampled scan
};

class VillSlamNode : public rclcpp::Node
{
public:
    VillSlamNode() : Node("vill_slam_node")
    {
        declareParameters();

        // Get parameters
        environment_mode_ = static_cast<EnvironmentMode>(
            this->get_parameter("environment_mode").as_int());
        initial_pipe_radius_ = this->get_parameter("initial_pipe_radius").as_double();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        // Loop closure parameters
        kf_distance_thresh_ = this->get_parameter("keyframe_distance_threshold").as_double();
        kf_rotation_thresh_ = this->get_parameter("keyframe_rotation_threshold").as_double();
        loop_search_radius_ = this->get_parameter("loop_search_radius").as_double();
        loop_min_interval_ = static_cast<int>(
            this->get_parameter("loop_min_keyframe_interval").as_int());
        icp_fitness_thresh_ = this->get_parameter("icp_fitness_threshold").as_double();
        loop_scan_voxel_ = this->get_parameter("loop_scan_voxel_size").as_double();
        double lidar_rate_hz = this->get_parameter("lidar_process_rate_hz").as_double();
        lidar_process_interval_sec_ = (lidar_rate_hz > 0.0) ? (1.0 / lidar_rate_hz) : 0.0;
        last_lidar_process_time_ = std::chrono::steady_clock::time_point::min();
        loop_cooldown_kfs_ = static_cast<int>(
            this->get_parameter("loop_cooldown_keyframes").as_int());

        initializeComponents();
        createSubscribers();
        createPublishers();
        createServices();

        // Status + loop closure timers
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VillSlamNode::publishStatus, this));

        loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&VillSlamNode::detectLoopClosure, this));

        RCLCPP_INFO(this->get_logger(), "VILL-SLAM node initialized (with loop closure)");
        RCLCPP_INFO(this->get_logger(), "  Environment mode: %s",
            environment_mode_ == EnvironmentMode::PIPE ? "PIPE" :
            environment_mode_ == EnvironmentMode::CORRIDOR ? "CORRIDOR" : "AUTO");
#ifdef USE_GTSAM
        RCLCPP_INFO(this->get_logger(), "  GTSAM pose graph: ENABLED");
#else
        RCLCPP_WARN(this->get_logger(), "  GTSAM not available - loop closure disabled");
#endif
    }

private:
    void declareParameters()
    {
        this->declare_parameter("environment_mode", 3);
        this->declare_parameter("initial_pipe_radius", 0.3);
        this->declare_parameter("publish_tf", true);

        // Factor weights
        this->declare_parameter("cylinder_factor_weight", 10.0);
        this->declare_parameter("plane_factor_weight", 5.0);
        this->declare_parameter("imu_factor_weight", 100.0);
        this->declare_parameter("lidar_factor_weight", 1.0);

        // Frame IDs
        this->declare_parameter("world_frame", "map");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");

        // Topic names
        this->declare_parameter("imu_topic", "/zed_front/zed_node/imu/data");
        this->declare_parameter("lidar_topic", "/ouster/points");
        this->declare_parameter("fast_lio_odom_topic", "/Odometry");
        this->declare_parameter("laser_line_topic", "/vill_slam/laser_line");

        // Loop closure parameters
        this->declare_parameter("keyframe_distance_threshold", 0.5);
        this->declare_parameter("keyframe_rotation_threshold", 0.3);
        this->declare_parameter("loop_search_radius", 5.0);
        this->declare_parameter("loop_min_keyframe_interval", 30);
        this->declare_parameter("icp_fitness_threshold", 0.3);
        this->declare_parameter("loop_scan_voxel_size", 0.2);
        // LiDAR processing throttle: voxel filter only needed for keyframe
        // candidates. With 0.5m keyframe distance and 1 m/s speed, 1 Hz is
        // enough; 0 disables throttle.
        this->declare_parameter("lidar_process_rate_hz", 1.0);
        // Loop closure cooldown: 최근 루프 후 N개 키프레임이 추가되기 전까지
        // 새로운 루프 클로저 시도를 막아 false positive 폭주 방지
        this->declare_parameter("loop_cooldown_keyframes", 5);

        // Surface detection
        this->declare_parameter("cylinder_fit_threshold", 0.05);
        this->declare_parameter("plane_fit_threshold", 0.03);
        this->declare_parameter("min_detection_points", 50);
        this->declare_parameter("min_pipe_radius", 0.1);
        this->declare_parameter("max_pipe_radius", 3.0);
    }

    void initializeComponents()
    {
        // Surface processor
        SurfaceProcessor::Config proc_config;
        proc_config.min_radius = this->get_parameter("min_pipe_radius").as_double();
        proc_config.max_radius = this->get_parameter("max_pipe_radius").as_double();
        surface_processor_ = std::make_unique<SurfaceProcessor>(proc_config);

        if (environment_mode_ != EnvironmentMode::AUTO) {
            surface_processor_->setEnvironmentMode(environment_mode_);
        }

        // State
        current_pose_ = Eigen::Isometry3d::Identity();
        estimated_radius_ = initial_pipe_radius_;
        total_distance_ = 0.0;
        next_keyframe_id_ = 0;
        loop_closure_count_ = 0;

        // TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

#ifdef USE_GTSAM
        // iSAM2 initialization
        gtsam::ISAM2Params isam_params;
        isam_params.relinearizeThreshold = 0.1;
        isam_params.relinearizeSkip = 1;
        isam2_ = std::make_unique<gtsam::ISAM2>(isam_params);
#endif
    }

    void createSubscribers()
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic").as_string(),
            rclcpp::SensorDataQoS(),
            std::bind(&VillSlamNode::imuCallback, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("lidar_topic").as_string(),
            rclcpp::SensorDataQoS(),
            std::bind(&VillSlamNode::lidarCallback, this, std::placeholders::_1));

        fast_lio_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("fast_lio_odom_topic").as_string(),
            10,
            std::bind(&VillSlamNode::fastLioCallback, this, std::placeholders::_1));

        laser_line_sub_ = this->create_subscription<vill_slam_msgs::msg::LaserLine>(
            this->get_parameter("laser_line_topic").as_string(),
            10,
            std::bind(&VillSlamNode::laserLineCallback, this, std::placeholders::_1));
    }

    void createPublishers()
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/vill_slam/odometry", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/vill_slam/pose", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/vill_slam/path", 10);
        surface_pub_ = this->create_publisher<vill_slam_msgs::msg::SurfaceSection>(
            "/vill_slam/surface_section", 10);
        status_pub_ = this->create_publisher<vill_slam_msgs::msg::VillSlamStatus>(
            "/vill_slam/status", 10);
    }

    void createServices()
    {
        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/vill_slam/reset",
            std::bind(&VillSlamNode::resetCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        save_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/vill_slam/save_map",
            std::bind(&VillSlamNode::saveMapCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
    }

    // ================================================================
    // Callbacks
    // ================================================================

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_imu_ = msg;
        last_imu_time_ = rclcpp::Time(this->now().nanoseconds(), RCL_ROS_TIME);
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Update last_lidar_time always (used for sensor health)
        last_lidar_time_ = rclcpp::Time(this->now().nanoseconds(), RCL_ROS_TIME);

        // Throttle: voxel filter is expensive and only needed for keyframes.
        // Keyframe selection runs at ~0.5-1 Hz (distance threshold 0.5m,
        // typical speed ~1 m/s), so processing every LiDAR frame is wasteful.
        if (lidar_process_interval_sec_ > 0.0) {
            auto now = std::chrono::steady_clock::now();
            if (last_lidar_process_time_ != std::chrono::steady_clock::time_point::min()) {
                double elapsed = std::chrono::duration<double>(
                    now - last_lidar_process_time_).count();
                if (elapsed < lidar_process_interval_sec_) {
                    return;
                }
            }
            last_lidar_process_time_ = now;
        }

        // Store downsampled scan for keyframe creation
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *raw_cloud);

        // Downsample for loop closure matching
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(raw_cloud);
        voxel.setLeafSize(loop_scan_voxel_, loop_scan_voxel_, loop_scan_voxel_);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        voxel.filter(*filtered);

        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_scan_ = filtered;
        latest_scan_stamp_ = msg->header.stamp;
    }

    void fastLioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        Eigen::Isometry3d new_pose = Eigen::Isometry3d::Identity();
        new_pose.translation() = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        new_pose.linear() = q.toRotationMatrix();

        double delta_dist = (new_pose.translation() - current_odom_pose_.translation()).norm();
        total_distance_ += delta_dist;

        previous_odom_pose_ = current_odom_pose_;
        current_odom_pose_ = new_pose;

        // Try to create a new keyframe
        tryAddKeyframe(msg->header.stamp);

        // Publish corrected pose (optimized if available, otherwise raw)
        publishOdometry(msg->header.stamp);
        if (publish_tf_) {
            broadcastTF(msg->header.stamp);
        }
    }

    void laserLineCallback(const vill_slam_msgs::msg::LaserLine::SharedPtr msg)
    {
        if (!msg->is_valid) return;

        std::lock_guard<std::mutex> lock(state_mutex_);

        std::vector<Eigen::Vector3d> points;
        points.reserve(msg->points_3d.size());
        for (const auto& pt : msg->points_3d) {
            points.emplace_back(pt.x, pt.y, pt.z);
        }

        surface_processor_->processLaserLine(points);
        EnvironmentMode detected = surface_processor_->detectEnvironment();

        // Publish surface section
        vill_slam_msgs::msg::SurfaceSection section_msg;
        section_msg.header = msg->header;
        section_msg.environment_mode = static_cast<uint8_t>(detected);

        if (detected == EnvironmentMode::PIPE) {
            auto pipe = surface_processor_->getPipeCrossSection();
            section_msg.geometry_type = vill_slam_msgs::msg::SurfaceSection::GEOMETRY_CYLINDER;
            section_msg.radius = pipe.radius;
            section_msg.radius_variance = pipe.radius_variance;
            section_msg.eccentricity = pipe.eccentricity;
            section_msg.confidence = pipe.confidence;
            section_msg.is_valid = pipe.confidence > 0.5;

            if (pipe.confidence > 0.7) {
                estimated_radius_ = 0.9 * estimated_radius_ + 0.1 * pipe.radius;
            }

#ifdef USE_GTSAM
            // Add cylinder constraint to latest keyframe
            addGeometricConstraint(detected);
#endif
        }
        else if (detected == EnvironmentMode::CORRIDOR) {
            auto walls = surface_processor_->getWallPlanes();
            section_msg.geometry_type = vill_slam_msgs::msg::SurfaceSection::GEOMETRY_PLANE;
            if (!walls.empty()) {
                section_msg.plane_coefficients[0] = walls[0].normal.x();
                section_msg.plane_coefficients[1] = walls[0].normal.y();
                section_msg.plane_coefficients[2] = walls[0].normal.z();
                section_msg.plane_coefficients[3] = walls[0].distance;
                section_msg.confidence = walls[0].confidence;
                section_msg.is_valid = walls[0].confidence > 0.5;
            }

#ifdef USE_GTSAM
            addGeometricConstraint(detected);
#endif
        }

        surface_pub_->publish(section_msg);
    }

    // ================================================================
    // Keyframe management
    // ================================================================

    void tryAddKeyframe(const rclcpp::Time& stamp)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan;
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (!latest_scan_ || latest_scan_->empty()) return;
            scan = latest_scan_;
            latest_scan_ = nullptr;  // Consume
        }

        // Check if enough motion since last keyframe
        if (!keyframes_.empty()) {
            const auto& last_kf = keyframes_.back();
            double dist = (current_odom_pose_.translation() -
                          last_kf.odom_pose.translation()).norm();
            Eigen::AngleAxisd aa(
                last_kf.odom_pose.rotation().transpose() * current_odom_pose_.rotation());
            double angle = std::abs(aa.angle());

            if (dist < kf_distance_thresh_ && angle < kf_rotation_thresh_) {
                return;  // Not enough motion
            }
        }

        // Create keyframe
        Keyframe kf;
        kf.id = next_keyframe_id_++;
        kf.stamp = stamp;
        kf.odom_pose = current_odom_pose_;
        kf.optimized_pose = current_odom_pose_;  // Will be updated by optimizer
        kf.cloud = scan;

#ifdef USE_GTSAM
        addKeyframeToGraph(kf);
#endif

        keyframes_.push_back(std::move(kf));

        RCLCPP_DEBUG(this->get_logger(), "Keyframe %lu added (total: %zu)",
            kf.id, keyframes_.size());
    }

#ifdef USE_GTSAM
    void addKeyframeToGraph(const Keyframe& kf)
    {
        gtsam::NonlinearFactorGraph new_factors;
        gtsam::Values new_values;

        gtsam::Pose3 gtsam_pose = eigenToGtsam(kf.odom_pose);

        if (kf.id == 0) {
            // First keyframe: add prior
            auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
            new_factors.addPrior(X(0), gtsam_pose, prior_noise);
        } else {
            // Odometry constraint between consecutive keyframes
            const auto& prev_kf = keyframes_.back();
            gtsam::Pose3 prev_pose = eigenToGtsam(prev_kf.odom_pose);
            gtsam::Pose3 relative = prev_pose.between(gtsam_pose);

            auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
            new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                X(prev_kf.id), X(kf.id), relative, odom_noise);
        }

        new_values.insert(X(kf.id), gtsam_pose);

        // Incremental update
        isam2_->update(new_factors, new_values);
        isam2_->update();  // Extra iteration for convergence

        // Update all keyframe poses from optimized values
        updateOptimizedPoses();
    }

    void addGeometricConstraint(EnvironmentMode mode)
    {
        if (keyframes_.empty()) return;

        gtsam::NonlinearFactorGraph new_factors;
        gtsam::Values new_values;
        uint64_t kf_id = keyframes_.back().id;

        if (mode == EnvironmentMode::PIPE) {
            auto pipe = surface_processor_->getPipeCrossSection();
            if (pipe.confidence < 0.5) return;

            // Initialize radius variable if first time
            if (!radius_initialized_) {
                new_values.insert(R(0), gtsam::Vector1(estimated_radius_));
                auto radius_prior = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
                new_factors.addPrior(R(0), gtsam::Vector1(estimated_radius_), radius_prior);
                radius_initialized_ = true;
            }

            PipeCrossSection measured;
            measured.center = pipe.center;
            measured.radius = pipe.radius;
            measured.radius_variance = pipe.radius_variance;
            measured.axis_direction = pipe.axis_direction;
            measured.confidence = pipe.confidence;

            double weight = this->get_parameter("cylinder_factor_weight").as_double();
            auto factor = createCylinderFactor(X(kf_id), R(0), measured, weight);
            if (factor) new_factors.push_back(factor);
        }
        else if (mode == EnvironmentMode::CORRIDOR) {
            auto walls = surface_processor_->getWallPlanes();
            if (walls.empty() || walls[0].confidence < 0.5) return;

            // Initialize plane variable if first time
            if (!plane_initialized_) {
                Eigen::Vector4d plane_init;
                plane_init << walls[0].normal, walls[0].distance;
                new_values.insert(P(0), gtsam::Vector4(plane_init));
                auto plane_prior = gtsam::noiseModel::Isotropic::Sigma(4, 0.5);
                new_factors.addPrior(P(0), gtsam::Vector4(plane_init), plane_prior);
                plane_initialized_ = true;
            }

            WallPlane measured;
            measured.normal = walls[0].normal;
            measured.distance = walls[0].distance;
            measured.confidence = walls[0].confidence;

            double weight = this->get_parameter("plane_factor_weight").as_double();
            auto factor = createPlaneFactor(X(kf_id), P(0), measured, weight);
            if (factor) new_factors.push_back(factor);
        }

        if (!new_factors.empty()) {
            isam2_->update(new_factors, new_values);
            updateOptimizedPoses();
        }
    }
#endif

    // ================================================================
    // Loop Closure Detection
    // ================================================================

    void detectLoopClosure()
    {
#ifdef USE_GTSAM
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (keyframes_.size() < static_cast<size_t>(loop_min_interval_ + 5)) {
            return;
        }

        const auto& current_kf = keyframes_.back();

        // Cooldown: 최근 루프 클로저 후 N개 키프레임 추가될 때까지 시도 안 함
        // (false positive 폭주 방지)
        if (loop_cooldown_active_ &&
            (current_kf.id - last_loop_kf_id_) < static_cast<uint64_t>(loop_cooldown_kfs_)) {
            return;
        }
        loop_cooldown_active_ = false;

        // Search for loop candidates: nearby keyframes that are far in sequence
        for (size_t i = 0; i < keyframes_.size() - loop_min_interval_; ++i) {
            const auto& candidate = keyframes_[i];

            double dist = (current_kf.optimized_pose.translation() -
                          candidate.optimized_pose.translation()).norm();

            if (dist > loop_search_radius_) continue;

            // ICP scan matching
            Eigen::Isometry3d relative_pose;
            double fitness;
            if (performICP(current_kf, candidate, relative_pose, fitness)) {
                if (fitness < icp_fitness_thresh_) {
                    // Add loop closure constraint
                    addLoopClosureConstraint(current_kf.id, candidate.id, relative_pose);
                    loop_closure_count_++;
                    last_loop_kf_id_ = current_kf.id;
                    loop_cooldown_active_ = true;

                    RCLCPP_INFO(this->get_logger(),
                        "Loop closure detected! KF %lu <-> KF %lu (fitness: %.4f, total: %lu, cooldown %d kfs)",
                        current_kf.id, candidate.id, fitness, loop_closure_count_,
                        loop_cooldown_kfs_);

                    // Only one loop closure per cycle
                    break;
                }
            }
        }
#endif
    }

    bool performICP(const Keyframe& source, const Keyframe& target,
                    Eigen::Isometry3d& relative_pose, double& fitness)
    {
        if (!source.cloud || !target.cloud ||
            source.cloud->empty() || target.cloud->empty()) {
            return false;
        }

        // Transform source cloud to target frame using current estimate
        Eigen::Isometry3d init_guess =
            target.optimized_pose.inverse() * source.optimized_pose;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaxCorrespondenceDistance(1.0);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);

        icp.setInputSource(source.cloud);
        icp.setInputTarget(target.cloud);

        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned, init_guess.matrix().cast<float>());

        if (!icp.hasConverged()) {
            return false;
        }

        fitness = icp.getFitnessScore();
        Eigen::Matrix4f result = icp.getFinalTransformation();
        relative_pose = Eigen::Isometry3d(result.cast<double>());

        return true;
    }

#ifdef USE_GTSAM
    void addLoopClosureConstraint(uint64_t source_id, uint64_t target_id,
                                   const Eigen::Isometry3d& relative_pose)
    {
        gtsam::NonlinearFactorGraph new_factors;

        gtsam::Pose3 relative = eigenToGtsam(relative_pose);

        // Loop closure noise (tighter than odometry since ICP-verified)
        auto loop_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.03, 0.03, 0.03, 0.05, 0.05, 0.05).finished());

        // Use robust noise model to handle potential false positives
        auto robust_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.0), loop_noise);

        new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(target_id), X(source_id), relative, robust_noise);

        isam2_->update(new_factors);
        isam2_->update();
        isam2_->update();  // Multiple iterations for loop closure convergence

        updateOptimizedPoses();
    }

    void updateOptimizedPoses()
    {
        auto result = isam2_->calculateEstimate();

        for (auto& kf : keyframes_) {
            if (result.exists(X(kf.id))) {
                gtsam::Pose3 optimized = result.at<gtsam::Pose3>(X(kf.id));
                kf.optimized_pose = gtsamToEigen(optimized);
            }
        }

        // Update current corrected pose based on relative from latest keyframe
        if (!keyframes_.empty()) {
            const auto& last_kf = keyframes_.back();
            Eigen::Isometry3d odom_delta =
                last_kf.odom_pose.inverse() * current_odom_pose_;
            current_pose_ = last_kf.optimized_pose * odom_delta;
        }
    }

    // GTSAM ↔ Eigen conversions
    static gtsam::Pose3 eigenToGtsam(const Eigen::Isometry3d& pose) {
        return gtsam::Pose3(gtsam::Rot3(pose.rotation()), pose.translation());
    }

    static Eigen::Isometry3d gtsamToEigen(const gtsam::Pose3& pose) {
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        result.linear() = pose.rotation().matrix();
        result.translation() = pose.translation();
        return result;
    }
#endif

    // ================================================================
    // Publishing
    // ================================================================

    Eigen::Isometry3d getCorrectedPose()
    {
#ifdef USE_GTSAM
        if (!keyframes_.empty()) {
            const auto& last_kf = keyframes_.back();
            Eigen::Isometry3d odom_delta =
                last_kf.odom_pose.inverse() * current_odom_pose_;
            return last_kf.optimized_pose * odom_delta;
        }
#endif
        return current_odom_pose_;
    }

    void publishOdometry(const rclcpp::Time& stamp)
    {
        current_pose_ = getCorrectedPose();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = this->get_parameter("world_frame").as_string();
        odom_msg.child_frame_id = this->get_parameter("base_frame").as_string();

        odom_msg.pose.pose.position.x = current_pose_.translation().x();
        odom_msg.pose.pose.position.y = current_pose_.translation().y();
        odom_msg.pose.pose.position.z = current_pose_.translation().z();

        Eigen::Quaterniond q(current_pose_.rotation());
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();

        odom_pub_->publish(odom_msg);

        // PoseStamped
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = odom_msg.header;
        pose_msg.pose = odom_msg.pose.pose;
        pose_pub_->publish(pose_msg);

        // Optimized path
        publishPath(stamp);
    }

    void publishPath(const rclcpp::Time& stamp)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = this->get_parameter("world_frame").as_string();

        for (const auto& kf : keyframes_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = kf.optimized_pose.translation().x();
            ps.pose.position.y = kf.optimized_pose.translation().y();
            ps.pose.position.z = kf.optimized_pose.translation().z();
            Eigen::Quaterniond q(kf.optimized_pose.rotation());
            ps.pose.orientation.w = q.w();
            ps.pose.orientation.x = q.x();
            ps.pose.orientation.y = q.y();
            ps.pose.orientation.z = q.z();
            path_msg.poses.push_back(ps);
        }

        path_pub_->publish(path_msg);
    }

    void broadcastTF(const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = this->get_parameter("world_frame").as_string();
        tf.child_frame_id = this->get_parameter("odom_frame").as_string();

        tf.transform.translation.x = current_pose_.translation().x();
        tf.transform.translation.y = current_pose_.translation().y();
        tf.transform.translation.z = current_pose_.translation().z();

        Eigen::Quaterniond q(current_pose_.rotation());
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(tf);
    }

    void publishStatus()
    {
        vill_slam_msgs::msg::VillSlamStatus status;
        status.header.stamp = this->now();

        status.system_state = vill_slam_msgs::msg::VillSlamStatus::STATE_RUNNING;

        EnvironmentMode detected = surface_processor_->detectEnvironment();
        status.environment_mode = static_cast<uint8_t>(detected);

        // SLAM statistics
        status.num_keyframes = keyframes_.size();
        status.total_distance = total_distance_;
        status.session_distance = total_distance_;
        status.loop_closure_count = static_cast<float>(loop_closure_count_);

#ifdef USE_GTSAM
        // Factor graph size from iSAM2
        if (isam2_) {
            status.num_constraints = isam2_->getFactorsUnsafe().size();
            status.num_landmarks = isam2_->getLinearizationPoint().size();
        }
#endif

        // Estimated drift = norm difference between raw odom and corrected pose
        // for the latest keyframe (proxy for accumulated SLAM correction)
        if (!keyframes_.empty()) {
            const auto& last_kf = keyframes_.back();
            status.estimated_drift = static_cast<float>(
                (last_kf.odom_pose.translation() -
                 last_kf.optimized_pose.translation()).norm());
        }

        status.current_pose.header.stamp = this->now();
        status.current_pose.pose.position.x = current_pose_.translation().x();
        status.current_pose.pose.position.y = current_pose_.translation().y();
        status.current_pose.pose.position.z = current_pose_.translation().z();

        Eigen::Quaterniond q(current_pose_.rotation());
        status.current_pose.pose.orientation.w = q.w();
        status.current_pose.pose.orientation.x = q.x();
        status.current_pose.pose.orientation.y = q.y();
        status.current_pose.pose.orientation.z = q.z();

        status.estimated_pipe_radius = estimated_radius_;
        status.pipe_radius_confidence = surface_processor_->getPipeConfidence();

        // Sensor health
        uint16_t sensors_active = 0;
        if (last_imu_time_.nanoseconds() > 0) {
            try {
                auto now_ros = rclcpp::Time(this->now().nanoseconds(), RCL_ROS_TIME);
                if ((now_ros - last_imu_time_).seconds() < 1.0) {
                    sensors_active |= vill_slam_msgs::msg::VillSlamStatus::SENSOR_IMU;
                }
            } catch (...) {}
        }
        if (last_lidar_time_.nanoseconds() > 0) {
            try {
                auto now_ros = rclcpp::Time(this->now().nanoseconds(), RCL_ROS_TIME);
                if ((now_ros - last_lidar_time_).seconds() < 1.0) {
                    sensors_active |= vill_slam_msgs::msg::VillSlamStatus::SENSOR_LIDAR;
                }
            } catch (...) {}
        }
        status.sensors_active = sensors_active;
        status.sensors_healthy = sensors_active;

        status_pub_->publish(status);
    }

    // ================================================================
    // Services
    // ================================================================

    void resetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        current_pose_ = Eigen::Isometry3d::Identity();
        current_odom_pose_ = Eigen::Isometry3d::Identity();
        previous_odom_pose_ = Eigen::Isometry3d::Identity();
        total_distance_ = 0.0;
        next_keyframe_id_ = 0;
        loop_closure_count_ = 0;
        last_loop_kf_id_ = 0;
        loop_cooldown_active_ = false;
        estimated_radius_ = initial_pipe_radius_;
        keyframes_.clear();

#ifdef USE_GTSAM
        gtsam::ISAM2Params isam_params;
        isam_params.relinearizeThreshold = 0.1;
        isam_params.relinearizeSkip = 1;
        isam2_ = std::make_unique<gtsam::ISAM2>(isam_params);
        radius_initialized_ = false;
        plane_initialized_ = false;
#endif

        response->success = true;
        response->message = "VILL-SLAM state reset (including pose graph)";
        RCLCPP_INFO(this->get_logger(), "SLAM state reset");
    }

    void saveMapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        response->success = true;
        response->message = "Map save triggered";
        RCLCPP_INFO(this->get_logger(), "Map save requested (%zu keyframes)",
            keyframes_.size());
    }

    // ================================================================
    // Member variables
    // ================================================================

    // Parameters
    EnvironmentMode environment_mode_;
    double initial_pipe_radius_;
    bool publish_tf_;
    double kf_distance_thresh_;
    double kf_rotation_thresh_;
    double loop_search_radius_;
    int loop_min_interval_;
    double icp_fitness_thresh_;
    double loop_scan_voxel_;
    double lidar_process_interval_sec_ = 0.0;
    std::chrono::steady_clock::time_point last_lidar_process_time_;
    int loop_cooldown_kfs_ = 5;
    uint64_t last_loop_kf_id_ = 0;
    bool loop_cooldown_active_ = false;

    // Components
    std::unique_ptr<SurfaceProcessor> surface_processor_;

    // State
    std::mutex state_mutex_;
    std::mutex cloud_mutex_;
    Eigen::Isometry3d current_pose_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d current_odom_pose_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d previous_odom_pose_{Eigen::Isometry3d::Identity()};
    double estimated_radius_ = 0.3;
    double total_distance_ = 0.0;
    uint64_t next_keyframe_id_ = 0;
    uint64_t loop_closure_count_ = 0;

    // Keyframes
    std::deque<Keyframe> keyframes_;

    // Latest scan for keyframe creation
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_scan_;
    rclcpp::Time latest_scan_stamp_{0, 0, RCL_ROS_TIME};

    // Sensor timestamps
    rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_lidar_time_{0, 0, RCL_ROS_TIME};
    sensor_msgs::msg::Imu::SharedPtr last_imu_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

#ifdef USE_GTSAM
    // Pose graph
    std::unique_ptr<gtsam::ISAM2> isam2_;
    bool radius_initialized_ = false;
    bool plane_initialized_ = false;
#endif

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fast_lio_sub_;
    rclcpp::Subscription<vill_slam_msgs::msg::LaserLine>::SharedPtr laser_line_sub_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<vill_slam_msgs::msg::SurfaceSection>::SharedPtr surface_pub_;
    rclcpp::Publisher<vill_slam_msgs::msg::VillSlamStatus>::SharedPtr status_pub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;

    // Timers
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
};

}  // namespace vill_slam

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vill_slam::VillSlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
