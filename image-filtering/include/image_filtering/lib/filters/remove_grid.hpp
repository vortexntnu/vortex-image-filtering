#ifndef IMAGE_FILTERING__LIB__FILTERS__REMOVE_GRID_HPP_
#define IMAGE_FILTERING__LIB__FILTERS__REMOVE_GRID_HPP_

#include <spdlog/spdlog.h>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <vector>

#include "abstract_filter_class.hpp"

namespace vortex::image_filtering {

struct RemoveGridParams {
    double inpaint_radius;
    int threshold_binary;
    bool use_binary_threshold;
    int rotation;
    int height;
    int width;
    int hsv_hue_low;
    int hsv_hue_high;
    int hsv_sat_low;
    int hsv_sat_high;
    int hsv_val_low;
    int hsv_val_high;
};

class RemoveGrid : public Filter {
   public:
    explicit RemoveGrid(RemoveGridParams params) : params_(params) {}

    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    RemoveGridParams params_;
};

inline void RemoveGrid::apply_filter(const cv::Mat& original,
                                     cv::Mat& filtered) const {
    if (original.empty()) {
        spdlog::error("RemoveGrid: input image is empty");
        return;
    }

    if (original.type() != CV_8UC3) {
        spdlog::error("RemoveGrid: expected CV_8UC3 image");
        original.copyTo(filtered);
        return;
    }

    // Rotate/crop to ROI
    int crop_w = std::min(params_.width, original.cols);
    int crop_h = std::min(params_.height, original.rows);

    if (crop_w != params_.width || crop_h != params_.height) {
        spdlog::warn(
            "RemoveGrid: requested crop size (width={}, height={}) does not "
            "fit "
            "within original image size (width={}, height={}); clamping to "
            "(width={}, height={})",
            params_.width, params_.height, original.cols, original.rows, crop_w,
            crop_h);
    }

    const cv::Point2f center_src(original.cols * 0.5f, original.rows * 0.5f);
    const cv::Point2f center_dst(crop_w * 0.5f, crop_h * 0.5f);

    cv::Mat M = cv::getRotationMatrix2D(center_src, params_.rotation, 1.0);
    if (M.type() != CV_64F)
        M.convertTo(M, CV_64F);

    M.at<double>(0, 2) += (center_dst.x - center_src.x);
    M.at<double>(1, 2) += (center_dst.y - center_src.y);

    cv::Mat cropped;
    cv::warpAffine(original, cropped, M, cv::Size(crop_w, crop_h),
                   cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // Detect yellow grid bars via HSV hue range (input is rgb8)
    cv::Mat hsv;
    cv::cvtColor(cropped, hsv, cv::COLOR_RGB2HSV);
    cv::Mat grid_mask;
    cv::inRange(hsv,
                cv::Scalar(params_.hsv_hue_low, params_.hsv_sat_low,
                           params_.hsv_val_low),
                cv::Scalar(params_.hsv_hue_high, params_.hsv_sat_high,
                           params_.hsv_val_high),
                grid_mask);

    // Dilate mask to fully cover grid bar edges
    static const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
    cv::Mat dilated;
    cv::dilate(grid_mask, dilated, kernel);

    // Prevent border leak
    dilated.row(0).setTo(0);
    dilated.row(dilated.rows - 1).setTo(0);
    dilated.col(0).setTo(0);
    dilated.col(dilated.cols - 1).setTo(0);

    if (cv::countNonZero(dilated) == 0) {
        original.copyTo(filtered);
        return;
    }

    // Optionally apply binary threshold before inpainting
    cv::Mat inpaint_src;
    if (params_.use_binary_threshold) {
        cv::Mat thresh_gray;
        apply_fixed_threshold(cropped, thresh_gray, params_.threshold_binary,
                              false);
        cv::cvtColor(thresh_gray, inpaint_src, cv::COLOR_GRAY2BGR);
    } else {
        inpaint_src = cropped;
    }

    // Inpaint grid
    cv::Mat inpainted;
    cv::inpaint(inpaint_src, dilated, inpainted, params_.inpaint_radius,
                cv::INPAINT_TELEA);

    // Warp inpainted ROI back into full-size image
    cv::Mat invM;
    cv::invertAffineTransform(M, invM);

    cv::Mat overlay_full;
    cv::warpAffine(inpainted, overlay_full, invM, original.size(),
                   cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    cv::Mat local_mask(inpainted.rows, inpainted.cols, CV_8U, cv::Scalar(255));
    cv::Mat mask_full;
    cv::warpAffine(local_mask, mask_full, invM, original.size(),
                   cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));

    filtered = original.clone();
    overlay_full.copyTo(filtered, mask_full);
}

}  // namespace vortex::image_filtering

#endif  // IMAGE_FILTERING__LIB__FILTERS__REMOVE_GRID_HPP_
