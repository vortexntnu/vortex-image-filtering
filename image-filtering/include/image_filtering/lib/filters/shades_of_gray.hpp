#ifndef IMAGE_FILTERING__LIB__FILTERS__SHADES_OF_GRAY_HPP_
#define IMAGE_FILTERING__LIB__FILTERS__SHADES_OF_GRAY_HPP_

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "abstract_filter_class.hpp"
/////////////////////////////
// Shades of Gray
/////////////////////////////
// Generalised Gray-World colour constancy (Finlayson & Trezzi, 2004). The
// illuminant of each channel is estimated with a Minkowski p-norm:
//     illum_c = ( mean( I_c^p ) )^(1/p)
// p = 1  -> classic Gray-World (channel mean)
// p -> inf -> White-Patch / Max-RGB (channel max)
// p = 6  -> common sweet spot, more robust than either extreme.
// Channels are then von-Kries scaled so their illuminant estimates match.
namespace vortex::image_filtering {
struct ShadesOfGrayParams {
    // Minkowski norm exponent. 1 = Gray-World, larger leans toward White-Patch.
    double norm_p;
    // Clamp on per-channel gain ([1/max_gain, max_gain]) for stability.
    double max_gain;
};

class ShadesOfGray : public Filter {
   public:
    explicit ShadesOfGray(ShadesOfGrayParams params) : filter_params_(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    ShadesOfGrayParams filter_params_;
};

inline void ShadesOfGray::apply_filter(const cv::Mat& original,
                                       cv::Mat& filtered) const {
    if (original.channels() != 3) {
        original.copyTo(filtered);
        return;
    }

    cv::Mat img;
    original.convertTo(img, CV_32F);

    const double p = std::max(this->filter_params_.norm_p, 1.0);
    const double eps = 1e-6;

    std::vector<cv::Mat> channels;
    cv::split(img, channels);

    // Per-channel Minkowski-norm illuminant estimate.
    double illum[3];
    for (int c = 0; c < 3; ++c) {
        cv::Mat powered;
        cv::pow(channels[c], p, powered);
        illum[c] = std::pow(std::max(cv::mean(powered)[0], eps), 1.0 / p);
    }
    const double illum_gray = (illum[0] + illum[1] + illum[2]) / 3.0;

    const double max_gain = std::max(this->filter_params_.max_gain, 1.0);
    const double min_gain = 1.0 / max_gain;

    for (int c = 0; c < 3; ++c) {
        double gain = illum_gray / std::max(illum[c], eps);
        gain = std::clamp(gain, min_gain, max_gain);
        channels[c] *= gain;
    }

    cv::merge(channels, img);

    // Preserve overall luminance so the colour correction doesn't dim/brighten.
    const cv::Scalar in_mean = cv::mean(original);
    const double in_gray = (in_mean[0] + in_mean[1] + in_mean[2]) / 3.0;
    const cv::Scalar out_mean = cv::mean(img);
    const double out_gray = (out_mean[0] + out_mean[1] + out_mean[2]) / 3.0;
    if (out_gray > eps) {
        img *= in_gray / out_gray;
    }

    img.convertTo(filtered, original.type());
}
}  // namespace vortex::image_filtering
#endif  // IMAGE_FILTERING__LIB__FILTERS__SHADES_OF_GRAY_HPP_
