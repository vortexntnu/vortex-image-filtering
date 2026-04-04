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
    double threshold_binary;
    double inpaint_radius;
    int rotation;
    int height;
    int width;
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

    // Rotate directly into cropped output
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

    const cv::Point2f center_src(
        original.cols * 0.5f, original.rows * 0.5f);  // center of source image
    const cv::Point2f center_dst(crop_w * 0.5f,
                                 crop_h * 0.5f);  // center of destination image

    cv::Mat M = cv::getRotationMatrix2D(center_src, params_.rotation,
                                        1.0);  // affine matrix
    // Ensure type for at<double>
    if (M.type() != CV_64F) {
        M.convertTo(M, CV_64F);
    }

    // Shift translation so original center maps to cropped center
    M.at<double>(0, 2) += (center_dst.x - center_src.x);
    M.at<double>(1, 2) += (center_dst.y - center_src.y);

    cv::Mat cropped;
    cv::warpAffine(original, cropped, M, cv::Size(crop_w, crop_h),
                   cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // Extract green grid mask
    cv::Mat cropped_f;
    cropped.convertTo(cropped_f, CV_32F, 1.0 / 255.0);

    std::vector<cv::Mat> ch(3);  // make a vector for BGR
    cv::split(cropped_f, ch);    // BGR

    cv::Mat sum = ch[0] + ch[1] + ch[2] + 1e-6f;  // avoid division by zero
    for (auto& c : ch)
        c /= sum;  // normalized color values

    // Otsu threshold on normalized green channel
    cv::Mat green8;
    cv::Mat greenTemp;
    greenTemp = ch[1] * 255;
    greenTemp.convertTo(green8, CV_8U);
    cv::Mat grid_mask;

    //cv::threshold(green8, grid_mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    //cv::adaptiveThreshold(green8, grid_mask, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 11, 2);
    cv::adaptiveThreshold(green8, grid_mask, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);

    static const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
    cv::Mat dilated;
    cv::dilate(grid_mask, dilated, kernel);

    // prevent border leak
    dilated.row(0).setTo(0);
    dilated.row(dilated.rows - 1).setTo(0);
    dilated.col(0).setTo(0);
    dilated.col(dilated.cols - 1).setTo(0);

    if (cv::countNonZero(dilated) == 0) {
        // If no grid detected, leave image unchanged
        original.copyTo(filtered);
        return;
    }

    // Inpaint grid
    cv::Mat inpainted;
    cv::inpaint(cropped, dilated, inpainted, params_.inpaint_radius,
                cv::INPAINT_TELEA);

    // Binary threshold (on cropped ROI)
    cv::Mat thresh_gray;
    apply_fixed_threshold(inpainted, thresh_gray, params_.threshold_binary,
                          false);

    cv::Mat thresh_bgr;
    cv::cvtColor(thresh_gray, thresh_bgr, cv::COLOR_GRAY2BGR);

    // Undo rotation & merge (using M)
    cv::Mat invM;
    cv::invertAffineTransform(M, invM);

    // Warp ROI result back into full-size overlay
    cv::Mat overlay_full;
    cv::warpAffine(thresh_bgr, overlay_full, invM, original.size(),
                   cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // Warp a mask the same way (so black pixels are copied too)
    cv::Mat local_mask(thresh_bgr.rows, thresh_bgr.cols, CV_8U,
                       cv::Scalar(255));
    cv::Mat mask_full;
    cv::warpAffine(local_mask, mask_full, invM, original.size(),
                   cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));

    // Merge into the original image
    filtered = original.clone();
    overlay_full.copyTo(filtered, mask_full);
}

}  // namespace vortex::image_filtering

#endif  // IMAGE_FILTERING__LIB__FILTERS__REMOVE_GRID_HPP_
