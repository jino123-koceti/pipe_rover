// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause
// Ported to ROS2

#include "coin_lio/image_processing.h"

#include <stdexcept>
#include "coin_lio/timing.h"

ImageProcessor::ImageProcessor(rclcpp::Node::SharedPtr nh, std::shared_ptr<Projector> projector ,
    std::shared_ptr<FeatureManager> manager) : projector_(projector) {
    try {
        loadParameters(nh);
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(nh->get_logger(), "%s", e.what());
        exit(1);
    }

    rows_ = projector->rows();
    cols_ = projector->cols();
    min_range_ = manager->minRange();
    max_range_ = manager->maxRange();

    kernel_dx_ = cv::Mat::zeros(1, 3, CV_32F);
    kernel_dy_ = cv::Mat::zeros(3, 1, CV_32F);
    kernel_dx_.at<float>(0, 0) = -0.5;
    kernel_dx_.at<float>(0, 2) = 0.5;
    kernel_dy_.at<float>(0, 0) = -0.5;
    kernel_dy_.at<float>(2, 0) = 0.5;

    int k_size = manager->patchSize() + erosion_margin_;
    kernel_erosion_ = cv::Mat::ones(k_size, k_size, CV_32FC1);
};

void ImageProcessor::loadParameters(rclcpp::Node::SharedPtr nh) {
    auto declare_if_not = [&](const std::string& name, auto default_val) {
        if (!nh->has_parameter(name)) {
            nh->declare_parameter(name, default_val);
        }
    };

    declare_if_not("image.reflectivity", false);
    declare_if_not("image.line_removal", true);
    declare_if_not("image.brightness_filter", true);
    declare_if_not("image.blur", true);
    declare_if_not("image.intensity_scale", 0.25);
    declare_if_not("image.erosion_margin", 2);
    declare_if_not("image.window", std::vector<int64_t>{41, 7});
    declare_if_not("image.masks", std::vector<int64_t>());
    declare_if_not("image.highpass", std::vector<double>());
    declare_if_not("image.lowpass", std::vector<double>());

    reflectivity_ = nh->get_parameter("image.reflectivity").as_bool();
    remove_lines_ = nh->get_parameter("image.line_removal").as_bool();
    brightness_filter_ = nh->get_parameter("image.brightness_filter").as_bool();
    blur_ = nh->get_parameter("image.blur").as_bool();
    intensity_scale_ = nh->get_parameter("image.intensity_scale").as_double();
    erosion_margin_ = static_cast<int>(nh->get_parameter("image.erosion_margin").as_int());

    auto window = nh->get_parameter("image.window").as_integer_array();
    if (window.size() != 2) {
        throw std::runtime_error("Invalid window size");
    }

    auto masks = nh->get_parameter("image.masks").as_integer_array();
    if (masks.size() % 4 != 0) {
        throw std::runtime_error("Invalid masks, number of elements must be a multiple of 4");
    }

    for (size_t i = 0; i < masks.size()/4; i++) {
        masks_.push_back(cv::Rect(masks[i*4], masks[i*4 +1], masks[i*4 +2],masks[i*4 +3]));
    }

    window_size_ = cv::Size(window[0], window[1]);
    auto hpf = nh->get_parameter("image.highpass").as_double_array();
    high_pass_fir_ = cv::Mat(hpf).clone();
    auto lpf = nh->get_parameter("image.lowpass").as_double_array();
    low_pass_fir_ = cv::Mat(lpf).clone();
}

void ImageProcessor::createImages(LidarFrame& frame) {
    timing::Timer projection_timer("pre/projection");
    projector_->createImages(frame);
    projection_timer.Stop();

    // Debug: 이미지 유효성 확인
    static bool first = true;
    if (first) {
        int nonzero_int = cv::countNonZero(frame.img_intensity > 0);
        int nonzero_range = cv::countNonZero(frame.img_range > 0);
        int valid_idx = 0;
        for (int v = 0; v < frame.img_idx.rows; v++)
            for (int u = 0; u < frame.img_idx.cols; u++)
                if (frame.img_idx.ptr<int>(v)[u] >= 0) valid_idx++;
        std::cout << "[ImageProc] img_intensity nonzero=" << nonzero_int
                  << " img_range nonzero=" << nonzero_range
                  << " img_idx valid=" << valid_idx
                  << " hpf_size=" << high_pass_fir_.total()
                  << " lpf_size=" << low_pass_fir_.total() << std::endl;
        first = false;
    }

    if (!reflectivity_) {
        frame.img_intensity *= intensity_scale_;
    }

    timing::Timer img_process_timer("pre/image_process");

    if (remove_lines_ && high_pass_fir_.total() > 0 && low_pass_fir_.total() > 0) {
        removeLines(frame.img_intensity);
    }

    if (brightness_filter_) {
        filterBrightness(frame.img_intensity);
    }

    if (blur_) {
        cv::Mat img_blur;
        cv::GaussianBlur(frame.img_intensity, img_blur, cv::Size(3,3), 0);
        frame.img_intensity = img_blur;
    }

    cv::threshold(frame.img_intensity, frame.img_intensity, 255., 255., cv::THRESH_TRUNC);

    frame.img_intensity.convertTo(frame.img_photo_u8, CV_8UC1, 1);

    cv::filter2D(frame.img_intensity, frame.img_dx, CV_32F , kernel_dx_);
    cv::filter2D(frame.img_intensity, frame.img_dy, CV_32F , kernel_dy_);

    createMask(frame.img_range, frame.img_mask);

    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int v = 0; v < rows_; v++) {
        for (int u = 0; u < cols_; u++) {
            const int idx = frame.img_idx.ptr<int>(v)[u];
            if (idx == -1) continue;
            frame.points_corrected->points[idx].normal_z = frame.img_intensity.ptr<float>(v)[u];
        }
    }

    img_process_timer.Stop();
}

void ImageProcessor::removeLines(cv::Mat& img) {
    cv::Mat im_hpf;
    cv::filter2D(img, im_hpf, CV_32F , high_pass_fir_);
    cv::Mat im_lpf;
    cv::filter2D(im_hpf, im_lpf, CV_32F , low_pass_fir_.t());
    img -= im_lpf;
    img.setTo(0, img < 0);
}

void ImageProcessor::filterBrightness(cv::Mat& img) {
    cv::Mat brightness;
    cv::blur(img, brightness, window_size_);
    brightness += 1;
    cv::Mat normalized_img = (140.*img / brightness);
    img = normalized_img;
}

void ImageProcessor::createMask(const cv::Mat& range_img, cv::Mat& mask) {
    mask = cv::Mat::ones(range_img.rows, range_img.cols, CV_8UC1);
    for (auto& mask_rect : masks_) {
        mask(mask_rect) = 0;
    }

    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int v = 0; v < rows_; v++) {
        for (int u = 0; u < cols_; u++) {
            const float r = range_img.ptr<float>(v)[u];
            if (r < min_range_ || r > max_range_) {
                mask.ptr<uchar>(v)[u] = 0u;
            }
        }
    }
    cv::Mat img_eroded;
    cv::erode(mask, img_eroded, kernel_erosion_);
    mask = img_eroded;
}
