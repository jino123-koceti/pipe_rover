// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause
// Ported to ROS2

#include "coin_lio/projector.h"
#include <stdexcept>

#define DUPLICATE_POINTS 10

Projector::Projector(rclcpp::Node::SharedPtr nh) {
    try {
        loadParameters(nh);
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(nh->get_logger(), "%s", e.what());
        exit(1);
    }

    for (auto& angle : elevation_angles_) {
        angle *= M_PI/180.;
    }

    double fy = - static_cast<double>(rows_) / fabs(elevation_angles_[0] -
        elevation_angles_[elevation_angles_.size() - 1]);
    double fx = - static_cast<double>(cols_) / (2 * M_PI);
    double cy = rows_ / 2;
    double cx = cols_ / 2;

    K_ << fx, 0, cx,
          0, fy, cy,
          0, 0, 1;

    // mm to m
    beam_offset_m_ *= 1e-3;

    // bookkeeping for lookup from idx to row and column
    idx_to_v_ = std::vector<int>(rows_*cols_, 0);
    idx_to_u_ = std::vector<int>(rows_*cols_, 0);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            auto idx = indexFromPixel(V2D(i,j));
            idx_to_v_[idx] = i;
            idx_to_u_[idx] = j - u_shift_;
            if (idx_to_u_[idx] < 0) {
             idx_to_u_[idx] += cols_;
            }
            if (idx_to_u_[idx] >= (int)cols_) {
             idx_to_u_[idx] -= cols_;
            }
        }
    }
};

void Projector::loadParameters(rclcpp::Node::SharedPtr nh) {
    // These parameters come from Ouster metadata
    // In ROS2, they need to be loaded from config or metadata file

    // Try to get from parameters (set via config yaml)
    auto declare_if_not = [&](const std::string& name, auto default_val) {
        if (!nh->has_parameter(name)) {
            nh->declare_parameter(name, default_val);
        }
    };

    declare_if_not("ouster.pixels_per_column", 128);
    declare_if_not("ouster.columns_per_frame", 1024);
    declare_if_not("ouster.lidar_origin_to_beam_origin_mm", 15.806);
    declare_if_not("ouster.pixel_shift_by_row", std::vector<int64_t>());
    declare_if_not("ouster.beam_altitude_angles", std::vector<double>());
    declare_if_not("image.u_shift", 0);
    declare_if_not("image.destagger", true);

    rows_ = static_cast<size_t>(nh->get_parameter("ouster.pixels_per_column").as_int());
    cols_ = static_cast<size_t>(nh->get_parameter("ouster.columns_per_frame").as_int());
    beam_offset_m_ = nh->get_parameter("ouster.lidar_origin_to_beam_origin_mm").as_double();

    auto offset_lut_int64 = nh->get_parameter("ouster.pixel_shift_by_row").as_integer_array();
    offset_lut_.resize(offset_lut_int64.size());
    for (size_t i = 0; i < offset_lut_int64.size(); ++i) {
        offset_lut_[i] = static_cast<int>(offset_lut_int64[i]);
    }

    elevation_angles_ = nh->get_parameter("ouster.beam_altitude_angles").as_double_array();
    u_shift_ = static_cast<int>(nh->get_parameter("image.u_shift").as_int());
    destagger_ = nh->get_parameter("image.destagger").as_bool();

    if (elevation_angles_.empty() || offset_lut_.empty()) {
        throw std::runtime_error("Missing Ouster metadata parameters (beam_altitude_angles, pixel_shift_by_row)");
    }
}

size_t Projector::vectorIndexFromRowCol(const size_t row, const size_t col) const {
    return (row * cols_ + col) * DUPLICATE_POINTS;
}

void Projector::createImages(LidarFrame& frame) const {
    frame.img_intensity = cv::Mat::zeros(rows_, cols_, CV_32FC1);
    frame.img_range = cv::Mat::zeros(rows_, cols_, CV_32FC1);
    frame.img_idx = cv::Mat::ones(rows_, cols_, CV_32SC1) * (-1);
    frame.proj_idx = std::vector<int>(rows_*cols_*DUPLICATE_POINTS, 0);

    std::vector<int> idx_to_vk(frame.points_corrected->points.size(), -1);
    std::vector<int> idx_to_uk(frame.points_corrected->points.size(), -1);

    // Create a projected image from the undistorted point cloud
    #ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
    #endif
    for (size_t j = 0; j < frame.points_corrected->points.size(); ++j) {
        const V3D p_Lk = frame.points_corrected->points[j].getVector3fMap().cast<double>();
        V2D px_k;
        if (!projectPoint(p_Lk, px_k)) continue;
        idx_to_uk[j] = std::round(px_k(0));
        idx_to_vk[j] = std::round(px_k(1));
    }

    size_t lut_size = idx_to_v_.size();  // rows_ * cols_

    // Debug: first frame only
    static bool first_frame = true;
    if (first_frame && frame.points_corrected->points.size() > 0) {
        int nx0 = static_cast<int>(frame.points_corrected->points[0].normal_x);
        int nx_last = static_cast<int>(frame.points_corrected->points.back().normal_x);
        std::cout << "[Projector] points=" << frame.points_corrected->points.size()
                  << " lut_size=" << lut_size
                  << " normal_x[0]=" << nx0
                  << " normal_x[last]=" << nx_last
                  << " rows=" << rows_ << " cols=" << cols_ << std::endl;
        first_frame = false;
    }

    for (size_t j = 0; j < frame.points_corrected->points.size(); ++j) {
        int idx = static_cast<int>(frame.points_corrected->points[j].normal_x);
        if (idx < 0 || idx >= (int)lut_size) continue;
        int v_i = idx_to_v_[idx];
        int u_i = idx_to_u_[idx];
        if (v_i < 0 || v_i >= (int)rows_ || u_i < 0 || u_i >= (int)cols_) continue;
        frame.img_range.ptr<float>(v_i)[u_i] = frame.points_corrected->points[j].normal_y;
        frame.img_intensity.ptr<float>(v_i)[u_i] = frame.points_corrected->points[j].intensity;
        frame.img_idx.ptr<int>(v_i)[u_i] = j;
    }

    for (size_t j = 0; j < frame.points_corrected->points.size(); ++j) {
        int u_k = idx_to_uk[j];
        int v_k = idx_to_vk[j];
        if (u_k < 0 || v_k < 0) continue;
        size_t start_idx = vectorIndexFromRowCol(v_k, u_k);
        size_t offset = frame.proj_idx[start_idx] + 1;
        if (offset >= DUPLICATE_POINTS) continue;
        size_t idx = start_idx + offset;
        frame.proj_idx[idx] = j;
        frame.proj_idx[start_idx] = offset;
    }
}

size_t Projector::indexFromPixel(const V2D& px) const {
    const int vv = (int(px(1)) + cols_ - offset_lut_[int(px(0))]) % cols_;
    const size_t index = px(0)* cols_ + (destagger_ ? vv : int(px(1)));
    return index;
};

bool Projector::projectPoint(const V3D& point, V2D& uv) const {
    const double L = sqrt(point.x()*point.x() + point.y()*point.y()) - beam_offset_m_;
    const double R = sqrt(point.z()*point.z() + L*L);
    const double phi = atan2(point.y(), point.x());
    const double theta = asin(point.z()/R);
    uv.x() = K_(0, 0) * phi + K_(0, 2);

    if (theta > elevation_angles_[0]) {
        uv.y() = 0;
        return false;
    } else if (theta < elevation_angles_[rows_ - 1]) {
        uv.y() = rows_ - 1;
        return false;
    }

    auto greater = (std::upper_bound(elevation_angles_.rbegin(), elevation_angles_.rend(), theta) + 1).base();
    auto smaller = greater + 1;
    if (greater == elevation_angles_.end()) {
        uv.y() = rows_ - 1;
    } else {
        uv.y() = std::distance(elevation_angles_.begin(), greater);
        uv.y() += (*greater - theta) / (*greater - *smaller);
    }

    return isFOV(uv);
}

bool Projector::isFOV(const V2D& uv) const {
    return (uv.x() >= 0 && uv.x() <= (int)cols_ - 1 && uv.y() >= 0 && uv.y() <= (int)rows_ - 1);
}

void Projector::projectionJacobian(const V3D& p, Eigen::MatrixXd& du_dp) const {
    double rxy = p.head<2>().norm();
    double L = rxy - beam_offset_m_;
    double R2 = L*L + p.z()*p.z();
    double irxy = 1./rxy;
    double irxy2 = irxy * irxy;
    double fx_irxy2 = K_(0,0) * irxy2;

    du_dp = Eigen::MatrixXd::Zero(2,3);
    du_dp << -fx_irxy2 * p.y(), fx_irxy2 * p.x(), 0,
        -K_(1,1)*p.x()*p.z()/(L*R2), -K_(1,1)*p.y()*p.z()/(L*R2), K_(1,1)*L/R2;
}

bool Projector::projectUndistortedPoint(const LidarFrame& frame,const V3D& p_L_k, V3D& p_L_i, V2D& uv,
    int& distortion_idx, bool round) const {
    V2D uv_k;
    if (!projectPoint(p_L_k, uv_k)) {
        return false;
    }

    if (round) {
        uv_k(0) = std::round(uv_k(0));
        uv_k(1) = std::round(uv_k(1));
    }

    distortion_idx = -1;
    int row = uv_k(1);
    int col = uv_k(0);

    int idx = vectorIndexFromRowCol(row, col);
    if (frame.proj_idx[idx] == 0) {
        row = 0;
        while (row < frame.img_intensity.rows) {
            idx = vectorIndexFromRowCol(row, col);
            if (frame.proj_idx[idx] > 0) {
                break;
            } else {
                ++row;
            }
        }
    }

    if (row >= frame.img_intensity.rows) {
        return false;
    }

    if(frame.proj_idx[idx] > 1) {
        float min_dist = std::numeric_limits<float>::max();
        for (int i = 1; i <= frame.proj_idx[idx]; i++) {
            const int j = frame.proj_idx[idx + i];
            const V3D p_cand = frame.points_corrected->points[j].getVector3fMap().cast<double>();
            const V3D diff = p_L_k - p_cand;
            const float dist = diff.norm();
            if (dist < min_dist) {
                min_dist = dist;
                distortion_idx = j;
            }
        }
    } else {
        distortion_idx = frame.proj_idx[idx + 1];
    }

    if (distortion_idx < 0) {
        return false;
    }

    const M4D& T_Li_Lk = frame.T_Li_Lk_vec[frame.vec_idx[distortion_idx]];
    p_L_i = T_Li_Lk.block<3,3>(0,0) * p_L_k + T_Li_Lk.block<3,1>(0,3);

    if (!projectPoint(p_L_i, uv)) {
        return false;
    }

    return true;
}
