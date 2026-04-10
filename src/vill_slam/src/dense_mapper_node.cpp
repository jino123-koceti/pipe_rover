/**
 * @file dense_mapper_node.cpp
 * @brief Dense RGB-D mapping for pipe/corridor environments
 *
 * Accumulates LiDAR point clouds colored by ZED camera projection.
 * Subscribes to VILL-SLAM optimized odometry for pose-corrected mapping.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <mutex>
#include <deque>
#include <chrono>

class DenseMapperNode : public rclcpp::Node
{
public:
    DenseMapperNode() : Node("dense_mapper_node")
    {
        // Parameters
        this->declare_parameter("voxel_size", 0.02);
        this->declare_parameter("max_range", 10.0);
        this->declare_parameter("min_range", 0.5);
        this->declare_parameter("save_path", "/home/test/vill_slam_maps/");
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("auto_save_interval", 0);
        // Throttle: 매 LiDAR 프레임마다 카메라 투영하는 비용이 큼.
        // Map 시각화 용도이므로 1-2 Hz면 충분. (LiDAR 처리 부하 분산)
        this->declare_parameter("process_rate_hz", 1.5);

        // Camera-LiDAR extrinsic: transform from LiDAR frame to camera frame
        // Default: camera is 0.3m forward, 0.3m below LiDAR (approximate)
        this->declare_parameter("T_cam_lidar", std::vector<double>{
            0.0, -1.0,  0.0,  0.0,
            0.0,  0.0, -1.0, -0.3,
            1.0,  0.0,  0.0, -0.3,
            0.0,  0.0,  0.0,  1.0
        });

        // Image topic (ZED X front camera)
        this->declare_parameter("image_topic",
            std::string("/zed_front/zed_node/rgb/image_rect_color"));
        this->declare_parameter("camera_info_topic",
            std::string("/zed_front/zed_node/rgb/camera_info"));

        voxel_size_ = this->get_parameter("voxel_size").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        min_range_ = this->get_parameter("min_range").as_double();
        save_path_ = this->get_parameter("save_path").as_string();

        // Throttle interval (seconds between processed frames)
        double rate_hz = this->get_parameter("process_rate_hz").as_double();
        process_interval_sec_ = (rate_hz > 0.0) ? (1.0 / rate_hz) : 0.0;
        last_process_time_ = std::chrono::steady_clock::time_point::min();

        // Load extrinsic
        auto T_vec = this->get_parameter("T_cam_lidar").as_double_array();
        if (T_vec.size() == 16) {
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    T_cam_lidar_(i, j) = T_vec[i * 4 + j];
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid T_cam_lidar size, using identity");
            T_cam_lidar_ = Eigen::Matrix4d::Identity();
        }

        // Initialize map
        accumulated_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGB>());

        // Subscribers
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", rclcpp::SensorDataQoS(),
            std::bind(&DenseMapperNode::cloudCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vill_slam/odometry", 10,
            std::bind(&DenseMapperNode::odomCallback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            this->get_parameter("image_topic").as_string(),
            rclcpp::SensorDataQoS(),
            std::bind(&DenseMapperNode::imageCallback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            this->get_parameter("camera_info_topic").as_string(),
            rclcpp::SensorDataQoS(),
            std::bind(&DenseMapperNode::cameraInfoCallback, this, std::placeholders::_1));

        // Publishers
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/vill_slam/dense_map", 10);
        local_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/vill_slam/local_map", 10);

        // Services
        save_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/vill_slam/save_dense_map",
            std::bind(&DenseMapperNode::saveMapCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        clear_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/vill_slam/clear_map",
            std::bind(&DenseMapperNode::clearMapCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Map publish timer (1 Hz)
        map_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DenseMapperNode::publishMap, this));

        RCLCPP_INFO(this->get_logger(), "Dense mapper initialized (with RGB projection)");
        RCLCPP_INFO(this->get_logger(), "  Voxel size: %.3f m", voxel_size_);
        RCLCPP_INFO(this->get_logger(), "  Save path: %s", save_path_.c_str());
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        current_pose_ = Eigen::Isometry3d::Identity();
        current_pose_.translation() = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        current_pose_.linear() = q.toRotationMatrix();

        has_pose_ = true;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            std::lock_guard<std::mutex> lock(image_mutex_);
            last_image_ = cv_ptr->image.clone();
            has_image_ = true;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "cv_bridge error: %s", e.what());
        }
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (has_camera_info_) return;  // Only need once

        // Extract intrinsic matrix K (3x3)
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        img_width_ = msg->width;
        img_height_ = msg->height;
        has_camera_info_ = true;

        RCLCPP_INFO(this->get_logger(),
            "Camera intrinsics loaded: fx=%.1f fy=%.1f cx=%.1f cy=%.1f (%ux%u)",
            fx_, fy_, cx_, cy_, img_width_, img_height_);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!has_pose_) return;

        // Throttle: skip frames to keep processing rate at process_rate_hz
        // (per-point camera projection is expensive; map viz needs only ~1 Hz)
        if (process_interval_sec_ > 0.0) {
            auto now = std::chrono::steady_clock::now();
            if (last_process_time_ != std::chrono::steady_clock::time_point::min()) {
                double elapsed = std::chrono::duration<double>(
                    now - last_process_time_).count();
                if (elapsed < process_interval_sec_) {
                    return;
                }
            }
            last_process_time_ = now;
        }

        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *input_cloud);

        // Get current pose and image
        Eigen::Isometry3d pose;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            pose = current_pose_;
        }

        cv::Mat image;
        bool can_color = false;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            if (has_image_ && has_camera_info_ && !last_image_.empty()) {
                image = last_image_.clone();
                can_color = true;
            }
        }

        // Transform to world frame and colorize
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>());

        for (const auto& pt : input_cloud->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }

            double range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (range < min_range_ || range > max_range_) {
                continue;
            }

            // Transform to world frame
            Eigen::Vector3d p_local(pt.x, pt.y, pt.z);
            Eigen::Vector3d p_world = pose * p_local;

            pcl::PointXYZRGB colored_pt;
            colored_pt.x = p_world.x();
            colored_pt.y = p_world.y();
            colored_pt.z = p_world.z();

            // Try to get color from camera projection
            bool colored = false;
            if (can_color) {
                colored = projectAndColor(p_local, image, colored_pt);
            }

            if (!colored) {
                // Fallback: height-based coloring for visualization
                float h = std::clamp(static_cast<float>(p_local.z() + 1.0) / 3.0f, 0.0f, 1.0f);
                colored_pt.r = static_cast<uint8_t>(255 * (1.0f - h));
                colored_pt.g = static_cast<uint8_t>(255 * std::min(2.0f * h, 2.0f * (1.0f - h)));
                colored_pt.b = static_cast<uint8_t>(255 * h);
            }

            transformed_cloud->points.push_back(colored_pt);
        }

        // Add to accumulated map
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            *accumulated_cloud_ += *transformed_cloud;

            if (accumulated_cloud_->size() > 100000) {
                downsampleMap();
            }
        }

        // Publish local map
        sensor_msgs::msg::PointCloud2 local_msg;
        pcl::toROSMsg(*transformed_cloud, local_msg);
        local_msg.header.stamp = msg->header.stamp;
        local_msg.header.frame_id = this->get_parameter("map_frame").as_string();
        local_map_pub_->publish(local_msg);
    }

    /**
     * @brief Project a LiDAR point to camera image and retrieve color
     *
     * @param p_lidar Point in LiDAR frame
     * @param image Current camera image (BGR)
     * @param colored_pt Output point with RGB color set
     * @return true if point was successfully projected and colored
     */
    bool projectAndColor(const Eigen::Vector3d& p_lidar,
                         const cv::Mat& image,
                         pcl::PointXYZRGB& colored_pt)
    {
        // Transform LiDAR point to camera frame
        Eigen::Vector4d p_lidar_h(p_lidar.x(), p_lidar.y(), p_lidar.z(), 1.0);
        Eigen::Vector4d p_cam_h = T_cam_lidar_ * p_lidar_h;

        // Check if point is in front of camera
        if (p_cam_h.z() <= 0.1) return false;

        // Project to image plane
        double u = fx_ * p_cam_h.x() / p_cam_h.z() + cx_;
        double v = fy_ * p_cam_h.y() / p_cam_h.z() + cy_;

        // Check bounds
        int ui = static_cast<int>(std::round(u));
        int vi = static_cast<int>(std::round(v));
        if (ui < 0 || ui >= static_cast<int>(img_width_) ||
            vi < 0 || vi >= static_cast<int>(img_height_)) {
            return false;
        }

        // Sample color from image (BGR → RGB)
        const cv::Vec3b& bgr = image.at<cv::Vec3b>(vi, ui);
        colored_pt.r = bgr[2];
        colored_pt.g = bgr[1];
        colored_pt.b = bgr[0];

        return true;
    }

    void downsampleMap()
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(accumulated_cloud_);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(
            new pcl::PointCloud<pcl::PointXYZRGB>());
        voxel_filter.filter(*filtered);
        accumulated_cloud_ = filtered;

        RCLCPP_DEBUG(this->get_logger(), "Map downsampled: %zu points",
            accumulated_cloud_->size());
    }

    void publishMap()
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (accumulated_cloud_->empty()) return;

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*accumulated_cloud_, map_msg);
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = this->get_parameter("map_frame").as_string();
        map_pub_->publish(map_msg);
    }

    void saveMapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);

        if (accumulated_cloud_->empty()) {
            response->success = false;
            response->message = "Map is empty";
            return;
        }

        downsampleMap();

        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << save_path_ << "vill_slam_map_" << time_t << ".pcd";
        std::string filename = ss.str();

        try {
            pcl::io::savePCDFileBinary(filename, *accumulated_cloud_);
            response->success = true;
            response->message = "Map saved to: " + filename;
            RCLCPP_INFO(this->get_logger(), "Map saved: %s (%zu points)",
                filename.c_str(), accumulated_cloud_->size());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Save failed: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", e.what());
        }
    }

    void clearMapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        accumulated_cloud_->clear();
        response->success = true;
        response->message = "Map cleared";
        RCLCPP_INFO(this->get_logger(), "Map cleared");
    }

    // Parameters
    double voxel_size_;
    double max_range_;
    double min_range_;
    std::string save_path_;
    double process_interval_sec_ = 0.0;
    std::chrono::steady_clock::time_point last_process_time_;

    // Camera-LiDAR extrinsic
    Eigen::Matrix4d T_cam_lidar_ = Eigen::Matrix4d::Identity();

    // Camera intrinsics
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    uint32_t img_width_ = 0, img_height_ = 0;
    bool has_camera_info_ = false;

    // State
    std::mutex state_mutex_;
    std::mutex map_mutex_;
    std::mutex image_mutex_;
    Eigen::Isometry3d current_pose_{Eigen::Isometry3d::Identity()};
    bool has_pose_ = false;

    // Map
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;

    // Image for coloring
    cv::Mat last_image_;
    bool has_image_ = false;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_map_srv_;

    // Timer
    rclcpp::TimerBase::SharedPtr map_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DenseMapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
