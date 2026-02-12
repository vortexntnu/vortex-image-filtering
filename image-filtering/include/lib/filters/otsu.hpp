#ifndef LIB__FILTERS__OTSU_HPP_
#define LIB__FILTERS__OTSU_HPP_

#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>
#include "abstract_filter_class.hpp"
#include "lib/utilities.hpp"
/////////////////////////////
// Otsu Segmentation
/////////////////////////////
namespace vortex::image_filtering {
struct OtsuSegmentationParams {
    bool gamma_auto_correction;
    double gamma_auto_correction_weight;
    bool otsu_segmentation;
    double gsc_weight_r;
    double gsc_weight_g;
    double gsc_weight_b;
    int erosion_size;
    int dilation_size;
};

class OtsuSegmentation : public Filter {
   public:
    explicit OtsuSegmentation(OtsuSegmentationParams params)
        : filter_params_(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& output) const override;

   private:
    OtsuSegmentationParams filter_params_;
};

inline void OtsuSegmentation::apply_filter(const cv::Mat& original,
                                           cv::Mat& filtered) const {
    bool gamma_auto_correction = this->filter_params_.gamma_auto_correction;
    double gamma_auto_correction_weight =
        this->filter_params_.gamma_auto_correction_weight;

    bool otsu_segmentation = this->filter_params_.otsu_segmentation;

    if (original.type() == CV_8UC3) {  // if the image type is bgr8
        to_weighted_gray(original, filtered, this->filter_params_.gsc_weight_b,
                         this->filter_params_.gsc_weight_g,
                         this->filter_params_.gsc_weight_r);
    } else if (original.type() == CV_8UC1) {
        original.copyTo(filtered);
    }  // if its mono8
    else
        spdlog::error("your image type is not matching this filter");

    if (gamma_auto_correction) {
        apply_auto_gamma(filtered, gamma_auto_correction_weight);
    }

    if (otsu_segmentation) {
        apply_otsu(filtered, filtered, false, 255);

        // Apply erosion followed by dilation (opening)

        apply_erosion(filtered, filtered, this->filter_params_.erosion_size,
                      cv::MORPH_CROSS);
        apply_dilation(filtered, filtered, this->filter_params_.dilation_size,
                       cv::MORPH_CROSS);
    }
}
}  // namespace vortex::image_filtering
#endif  // LIB__FILTERS__OTSU_HPP_
