/**
 * @file line_laser_processor_node.cpp
 * @brief Line laser detection and 3D reconstruction node
 *
 * Detects 120-degree fan beam laser line in camera images and
 * reconstructs 3D points using triangulation.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <vill_slam_msgs/msg/laser_line.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <cmath>
#include <mutex>

class LineLaserProcessorNode : public rclcpp::Node
{
public:
    LineLaserProcessorNode() : Node("line_laser_processor")
    {
        // Declare parameters
        this->declare_parameter("camera_topic", "/vill_slam/laser_frame");
        this->declare_parameter("output_topic", "/vill_slam/laser_line");

        // HSV thresholds for red laser detection
        this->declare_parameter("hue_min", 0);
        this->declare_parameter("hue_max", 10);
        this->declare_parameter("hue_min2", 170);  // Red wraps around in HSV
        this->declare_parameter("hue_max2", 180);
        this->declare_parameter("saturation_min", 100);
        this->declare_parameter("value_min", 200);

        // Detection parameters
        this->declare_parameter("min_line_points", 50);
        this->declare_parameter("gaussian_blur_size", 5);
        this->declare_parameter("morphology_size", 3);

        // Camera intrinsics (will be updated from camera_info)
        this->declare_parameter("fx", 732.766);
        this->declare_parameter("fy", 732.555);
        this->declare_parameter("cx", 960.0);
        this->declare_parameter("cy", 600.0);

        // Triangulation parameters
        this->declare_parameter("laser_camera_baseline", 0.15);  // 15cm
        this->declare_parameter("laser_angle_deg", 120.0);  // Fan angle

        // Get parameters
        camera_topic_ = this->get_parameter("camera_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

        hue_min_ = this->get_parameter("hue_min").as_int();
        hue_max_ = this->get_parameter("hue_max").as_int();
        hue_min2_ = this->get_parameter("hue_min2").as_int();
        hue_max2_ = this->get_parameter("hue_max2").as_int();
        sat_min_ = this->get_parameter("saturation_min").as_int();
        val_min_ = this->get_parameter("value_min").as_int();

        min_line_points_ = this->get_parameter("min_line_points").as_int();
        blur_size_ = this->get_parameter("gaussian_blur_size").as_int();
        morph_size_ = this->get_parameter("morphology_size").as_int();

        fx_ = this->get_parameter("fx").as_double();
        fy_ = this->get_parameter("fy").as_double();
        cx_ = this->get_parameter("cx").as_double();
        cy_ = this->get_parameter("cy").as_double();

        baseline_ = this->get_parameter("laser_camera_baseline").as_double();
        fan_angle_ = this->get_parameter("laser_angle_deg").as_double() * M_PI / 180.0;

        // Create subscribers and publishers
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, 10,
            std::bind(&LineLaserProcessorNode::imageCallback, this, std::placeholders::_1));

        laser_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "line_laser/laser1_state", 10,
            std::bind(&LineLaserProcessorNode::laserStateCallback, this, std::placeholders::_1));

        laser_line_pub_ = this->create_publisher<vill_slam_msgs::msg::LaserLine>(output_topic_, 10);

        // Debug image publisher
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("vill_slam/laser_debug", 10);

        RCLCPP_INFO(this->get_logger(), "Line laser processor initialized");
        RCLCPP_INFO(this->get_logger(), "  Camera topic: %s", camera_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
    }

private:
    void laserStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        laser_on_ = msg->data;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Only process if laser is on
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!laser_on_) {
                return;
            }
        }

        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;

            // Detect laser line
            std::vector<cv::Point2f> laser_points;
            float confidence = detectLaserLine(frame, laser_points);

            // Create output message
            vill_slam_msgs::msg::LaserLine laser_msg;
            laser_msg.header = msg->header;
            laser_msg.laser_id = 0;
            laser_msg.laser_on = true;

            if (laser_points.size() >= static_cast<size_t>(min_line_points_)) {
                laser_msg.is_valid = true;
                laser_msg.detection_confidence = confidence;
                laser_msg.num_points = laser_points.size();

                // Store image points
                for (const auto& pt : laser_points) {
                    geometry_msgs::msg::Point32 img_pt;
                    img_pt.x = pt.x;
                    img_pt.y = pt.y;
                    img_pt.z = 0.0f;
                    laser_msg.image_points.push_back(img_pt);
                }

                // Triangulate to 3D
                std::vector<cv::Point3f> points_3d;
                std::vector<float> depths;
                triangulatePoints(laser_points, points_3d, depths);

                for (size_t i = 0; i < points_3d.size(); ++i) {
                    geometry_msgs::msg::Point32 pt3d;
                    pt3d.x = points_3d[i].x;
                    pt3d.y = points_3d[i].y;
                    pt3d.z = points_3d[i].z;
                    laser_msg.points_3d.push_back(pt3d);
                    laser_msg.depths.push_back(depths[i]);
                }

                // Fit line and get endpoints
                if (points_3d.size() >= 2) {
                    cv::Vec4f line_params;
                    std::vector<cv::Point2f> pts_2d;
                    for (const auto& p : points_3d) {
                        pts_2d.emplace_back(p.x, p.y);
                    }
                    cv::fitLine(pts_2d, line_params, cv::DIST_L2, 0, 0.01, 0.01);

                    // Calculate line endpoints
                    float vx = line_params[0], vy = line_params[1];
                    float x0 = line_params[2], y0 = line_params[3];

                    float min_t = std::numeric_limits<float>::max();
                    float max_t = std::numeric_limits<float>::lowest();

                    for (const auto& p : pts_2d) {
                        float t = vx * (p.x - x0) + vy * (p.y - y0);
                        min_t = std::min(min_t, t);
                        max_t = std::max(max_t, t);
                    }

                    laser_msg.line_start.x = x0 + min_t * vx;
                    laser_msg.line_start.y = y0 + min_t * vy;
                    laser_msg.line_start.z = 0.0f;

                    laser_msg.line_end.x = x0 + max_t * vx;
                    laser_msg.line_end.y = y0 + max_t * vy;
                    laser_msg.line_end.z = 0.0f;

                    laser_msg.line_length = std::sqrt(
                        std::pow(laser_msg.line_end.x - laser_msg.line_start.x, 2) +
                        std::pow(laser_msg.line_end.y - laser_msg.line_start.y, 2));

                    // Calculate fit error
                    float total_error = 0.0f;
                    for (const auto& p : pts_2d) {
                        float dist = std::abs(vy * (p.x - x0) - vx * (p.y - y0));
                        total_error += dist * dist;
                    }
                    laser_msg.line_fit_error = std::sqrt(total_error / pts_2d.size());
                }
            } else {
                laser_msg.is_valid = false;
                laser_msg.detection_confidence = confidence;
                laser_msg.num_points = laser_points.size();
            }

            // Publish result
            laser_line_pub_->publish(laser_msg);

            // Publish debug image
            publishDebugImage(frame, laser_points, msg->header);

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    float detectLaserLine(const cv::Mat& frame, std::vector<cv::Point2f>& laser_points)
    {
        laser_points.clear();

        // Convert to HSV
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Apply Gaussian blur
        cv::Mat blurred;
        cv::GaussianBlur(hsv, blurred, cv::Size(blur_size_, blur_size_), 0);

        // Create mask for red color (two ranges due to HSV wraparound)
        cv::Mat mask1, mask2, mask;
        cv::inRange(blurred, cv::Scalar(hue_min_, sat_min_, val_min_),
                    cv::Scalar(hue_max_, 255, 255), mask1);
        cv::inRange(blurred, cv::Scalar(hue_min2_, sat_min_, val_min_),
                    cv::Scalar(hue_max2_, 255, 255), mask2);
        cv::bitwise_or(mask1, mask2, mask);

        // Morphological operations to clean up
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                    cv::Size(morph_size_, morph_size_));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        // Extract points from all contours
        for (const auto& contour : contours) {
            for (const auto& pt : contour) {
                laser_points.emplace_back(static_cast<float>(pt.x), static_cast<float>(pt.y));
            }
        }

        // Calculate subpixel positions using weighted centroid
        if (!laser_points.empty()) {
            // Use intensity-weighted centroid for each detected point
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            std::vector<cv::Point2f> refined_points;
            for (const auto& pt : laser_points) {
                int x = static_cast<int>(pt.x);
                int y = static_cast<int>(pt.y);

                // Skip if too close to edge
                if (x < 2 || x >= frame.cols - 2 || y < 2 || y >= frame.rows - 2) {
                    refined_points.push_back(pt);
                    continue;
                }

                // Compute weighted centroid in 5x5 window
                float sum_x = 0, sum_y = 0, sum_w = 0;
                for (int dy = -2; dy <= 2; ++dy) {
                    for (int dx = -2; dx <= 2; ++dx) {
                        float w = static_cast<float>(gray.at<uchar>(y + dy, x + dx));
                        sum_x += (x + dx) * w;
                        sum_y += (y + dy) * w;
                        sum_w += w;
                    }
                }

                if (sum_w > 0) {
                    refined_points.emplace_back(sum_x / sum_w, sum_y / sum_w);
                } else {
                    refined_points.push_back(pt);
                }
            }
            laser_points = refined_points;
        }

        // Calculate confidence based on point count and distribution
        float confidence = 0.0f;
        if (laser_points.size() >= static_cast<size_t>(min_line_points_)) {
            // Higher confidence for more points
            confidence = std::min(1.0f, static_cast<float>(laser_points.size()) / 500.0f);

            // Adjust based on spatial distribution
            if (laser_points.size() >= 2) {
                float min_x = std::numeric_limits<float>::max();
                float max_x = std::numeric_limits<float>::lowest();
                for (const auto& pt : laser_points) {
                    min_x = std::min(min_x, pt.x);
                    max_x = std::max(max_x, pt.x);
                }
                float spread = (max_x - min_x) / frame.cols;
                confidence *= std::min(1.0f, spread * 2.0f);
            }
        }

        return confidence;
    }

    void triangulatePoints(const std::vector<cv::Point2f>& image_points,
                          std::vector<cv::Point3f>& points_3d,
                          std::vector<float>& depths)
    {
        points_3d.clear();
        depths.clear();

        for (const auto& pt : image_points) {
            // Compute ray direction from camera center through pixel
            float ray_x = (pt.x - cx_) / fx_;
            float ray_y = (pt.y - cy_) / fy_;
            float ray_z = 1.0f;

            // Normalize ray
            float ray_norm = std::sqrt(ray_x * ray_x + ray_y * ray_y + ray_z * ray_z);
            ray_x /= ray_norm;
            ray_y /= ray_norm;
            ray_z /= ray_norm;

            // For 120-degree fan beam, compute depth based on pixel position
            // Assuming laser is mounted above camera, projecting downward
            float angle_from_center = std::atan2(pt.x - cx_, fx_);

            // Simple triangulation model for fan beam
            // Distance increases with angle from optical axis
            float half_fan = fan_angle_ / 2.0f;
            float normalized_angle = angle_from_center / half_fan;

            // Depth estimation (simplified model - should be calibrated)
            float depth = baseline_ / std::cos(angle_from_center);
            depth = std::max(0.1f, std::min(depth, 10.0f));  // Clamp to reasonable range

            // 3D point in camera frame
            float x = depth * ray_x;
            float y = depth * ray_y;
            float z = depth * ray_z;

            points_3d.emplace_back(x, y, z);
            depths.push_back(depth);
        }
    }

    void publishDebugImage(const cv::Mat& frame,
                          const std::vector<cv::Point2f>& laser_points,
                          const std_msgs::msg::Header& header)
    {
        cv::Mat debug_img = frame.clone();

        // Draw detected laser points
        for (const auto& pt : laser_points) {
            cv::circle(debug_img, cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)),
                      2, cv::Scalar(0, 255, 0), -1);
        }

        // Add info text
        std::string info = "Points: " + std::to_string(laser_points.size());
        cv::putText(debug_img, info, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

        // Convert and publish
        cv_bridge::CvImage cv_img;
        cv_img.header = header;
        cv_img.encoding = "bgr8";
        cv_img.image = debug_img;
        debug_image_pub_->publish(*cv_img.toImageMsg());
    }

    // Subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr laser_state_sub_;
    rclcpp::Publisher<vill_slam_msgs::msg::LaserLine>::SharedPtr laser_line_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

    // Parameters
    std::string camera_topic_;
    std::string output_topic_;
    int hue_min_, hue_max_, hue_min2_, hue_max2_;
    int sat_min_, val_min_;
    int min_line_points_;
    int blur_size_, morph_size_;
    double fx_, fy_, cx_, cy_;
    double baseline_;
    double fan_angle_;

    // State
    bool laser_on_ = false;
    std::mutex state_mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineLaserProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
