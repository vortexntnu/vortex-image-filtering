#ifndef IMAGE_FILTERING__LIB__FILTERS__REMOVE_GRID_HPP_
#define IMAGE_FILTERING__LIB__FILTERS__REMOVE_GRID_HPP_

#include <spdlog/spdlog.h>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <vector>

#include "abstract_filter_class.hpp"

/////////////////////////////
// Remove grid filter
/////////////////////////////
namespace vortex::image_filtering {

struct RemoveGridParams {
    double threshold_green;
    int threshold_binary;
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

    // ----------------------------
    // Rotation
    // ----------------------------
    cv::Point2f center(original.cols * 0.5f, original.rows * 0.5f);
    cv::Mat rotation_matrix =
        cv::getRotationMatrix2D(center, params_.rotation, 1.0);

    cv::Mat rotated;
    cv::warpAffine(original, rotated, rotation_matrix, original.size(),
                   cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // ----------------------------
    // Center crop
    // ----------------------------
    int crop_w = std::min(params_.width, rotated.cols);
    int crop_h = std::min(params_.height, rotated.rows);
    if (crop_w != params_.width || crop_h != params_.height) {
        spdlog::warn(
            "RemoveGrid: requested crop size (width={}, height={}) exceeds "
            "rotated image size (width={}, height={}; clamping to ) "
            "(width={}, height={})",
            params_.width,
            params_.height,
            rotated.cols,
            rotated.rows,
            crop_w,
            crop_h);
    }

    int x = (rotated.cols - crop_w) / 2;
    int y = (rotated.rows - crop_h) / 2;

    cv::Rect roi(x, y, crop_w, crop_h);
    cv::Mat cropped = rotated(roi);

    // ----------------------------
    // Extract green grid mask
    // ----------------------------
    cv::Mat cropped_f;
    cropped.convertTo(cropped_f, CV_32F, 1.0 / 255.0);

    std::vector<cv::Mat> ch(3);
    cv::split(cropped_f, ch);  // BGR

    cv::Mat sum = ch[0] + ch[1] + ch[2] + 1e-6f;
    for (auto& c : ch)
        c /= sum;

    cv::Mat grid_mask = (ch[1] > params_.threshold_green);

    cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
    cv::Mat dilated;
    cv::dilate(grid_mask, dilated, kernel);

    // prevent border leak
    dilated.row(0).setTo(0);
    dilated.row(dilated.rows - 1).setTo(0);
    dilated.col(0).setTo(0);
    dilated.col(dilated.cols - 1).setTo(0);

    // ----------------------------
    // Inpaint grid
    // ----------------------------
    cv::Mat inpainted;
    cv::inpaint(cropped, dilated, inpainted, params_.inpaint_radius,
                cv::INPAINT_TELEA);

    // ----------------------------
    // Binary threshold
    // ----------------------------
    cv::Mat thresh, thresh_bgr;
    apply_fixed_threshold(inpainted, thresh, this->params_.threshold_binary, false);    
    cv::cvtColor(thresh, thresh_bgr, cv::COLOR_GRAY2BGR);

    // ----------------------------
    // Undo rotation
    // ----------------------------
    cv::Mat roi_thresh_bgr = cv::Mat::zeros(original.size(), original.type());
    thresh_bgr.copyTo(roi_thresh_bgr(roi));

    cv::Mat roi_mask = cv::Mat::zeros(original.size(), CV_8U);
    roi_mask(roi).setTo(255);

    cv::Mat inv_rot;
    cv::invertAffineTransform(rotation_matrix, inv_rot);

    cv::Mat rotated_back, rotated_mask;
    cv::warpAffine(roi_thresh_bgr, rotated_back, inv_rot, original.size(),
                cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
    cv::warpAffine(roi_mask, rotated_mask, inv_rot, original.size(),
                cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));

    filtered = original.clone();
    rotated_back.copyTo(filtered, rotated_mask);
}

}  // namespace vortex::image_filtering

#endif  // IMAGE_FILTERING__LIB__FILTERS__REMOVE_GRID_HPP_
