#ifndef IMAGE_FILTERING__LIB__FILTERS__FIXED_GAIN_HPP_
#define IMAGE_FILTERING__LIB__FILTERS__FIXED_GAIN_HPP_

#include <opencv2/imgproc.hpp>
#include <vector>
#include "abstract_filter_class.hpp"
/////////////////////////////
// Fixed Gain White Balance
/////////////////////////////
// Applies constant, manually-calibrated per-channel gains (and an optional
// brightness offset). Unlike the adaptive methods, the correction does NOT
// depend on scene content, so the output colour profile is perfectly stable
// frame-to-frame. Calibrate the gains once for your water/lighting (e.g. so a
// known grey/white reference reads neutral) and the restored colours stay
// consistent - ideal for matching a fixed training distribution.
//     out_c = gain_c * in_c + offset
namespace vortex::image_filtering {
struct FixedGainParams {
    // Per-channel multipliers, in RGB order (matches the node's rgb8 encoding).
    double gain_r;
    double gain_g;
    double gain_b;
    // Constant added to every channel after scaling, in 8-bit units [0, 255].
    double offset;
};

class FixedGain : public Filter {
   public:
    explicit FixedGain(FixedGainParams params) : filter_params_(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    FixedGainParams filter_params_;
};

inline void FixedGain::apply_filter(const cv::Mat& original,
                                    cv::Mat& filtered) const {
    if (original.channels() != 3) {
        original.copyTo(filtered);
        return;
    }

    cv::Mat img;
    original.convertTo(img, CV_32F);

    // Channel order is RGB (the node delivers rgb8): 0 = R, 1 = G, 2 = B.
    std::vector<cv::Mat> channels;
    cv::split(img, channels);

    const double offset = this->filter_params_.offset;
    channels[0] = channels[0] * this->filter_params_.gain_r + offset;
    channels[1] = channels[1] * this->filter_params_.gain_g + offset;
    channels[2] = channels[2] * this->filter_params_.gain_b + offset;

    cv::merge(channels, img);

    // convertTo saturates to the output type's range (e.g. [0, 255] for CV_8U).
    img.convertTo(filtered, original.type());
}
}  // namespace vortex::image_filtering
#endif  // IMAGE_FILTERING__LIB__FILTERS__FIXED_GAIN_HPP_
