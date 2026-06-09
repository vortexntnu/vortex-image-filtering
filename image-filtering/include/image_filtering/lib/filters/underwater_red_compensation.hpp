#ifndef IMAGE_FILTERING__LIB__FILTERS__UNDERWATER_RED_COMPENSATION_HPP_
#define IMAGE_FILTERING__LIB__FILTERS__UNDERWATER_RED_COMPENSATION_HPP_

#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "abstract_filter_class.hpp"
/////////////////////////////
// Underwater Red-Channel Compensation + Gray World
/////////////////////////////
// Ancuti et al., "Color Balance and Fusion for Underwater Image Enhancement"
// (IEEE TIP 2018). Water attenuates red (and, deeper, blue) far more than
// green, so the red channel is nearly empty and a plain white balance just
// amplifies its noise. This first *reconstructs* the attenuated channel from
// the well-preserved green channel:
//     I_rc(x) = I_r(x) + alpha * (mean_g - mean_r) * (1 - I_r(x)) * I_g(x)
// (all in [0, 1]), optionally doing the same for blue, then applies a
// Gray-World white balance. This recovers far more believable colour than
// white-balancing the raw frame.
namespace vortex::image_filtering {
struct UnderwaterRedCompensationParams {
    // Strength of the green->red(/blue) compensation. 1.0 matches the paper.
    double alpha;
    // Also compensate the blue channel (useful in deeper / very blue water).
    bool compensate_blue;
    // Clamp on the Gray-World per-channel gain ([1/max_gain, max_gain]).
    double max_gain;
};

class UnderwaterRedCompensation : public Filter {
   public:
    explicit UnderwaterRedCompensation(UnderwaterRedCompensationParams params)
        : filter_params_(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    UnderwaterRedCompensationParams filter_params_;
};

inline void UnderwaterRedCompensation::apply_filter(const cv::Mat& original,
                                                    cv::Mat& filtered) const {
    if (original.channels() != 3) {
        original.copyTo(filtered);
        return;
    }

    // Normalise to [0, 1] for the compensation formula.
    cv::Mat img;
    original.convertTo(img, CV_32F, 1.0 / 255.0);

    // Channel order is RGB (the node delivers rgb8): 0 = R, 1 = G, 2 = B.
    std::vector<cv::Mat> channels;
    cv::split(img, channels);
    cv::Mat& r = channels[0];
    cv::Mat& g = channels[1];
    cv::Mat& b = channels[2];

    const double mean_r = cv::mean(r)[0];
    const double mean_g = cv::mean(g)[0];
    const double mean_b = cv::mean(b)[0];
    const double alpha = this->filter_params_.alpha;

    // Reconstruct red from green where red is attenuated relative to green.
    r = r + alpha * (mean_g - mean_r) * (1.0 - r).mul(g);
    if (this->filter_params_.compensate_blue) {
        b = b + alpha * (mean_g - mean_b) * (1.0 - b).mul(g);
    }
    for (cv::Mat& ch : channels) {
        cv::min(cv::max(ch, 0.0), 1.0, ch);
    }

    // Gray-World white balance on the compensated image (clamped + luminance
    // preserving for stability), mirroring gray_world_white_balancing.hpp.
    const double eps = 1e-6;
    const double max_gain = std::max(this->filter_params_.max_gain, 1.0);
    const double min_gain = 1.0 / max_gain;

    double comp_mean[3] = {cv::mean(r)[0], cv::mean(g)[0], cv::mean(b)[0]};
    const double mean_gray = (comp_mean[0] + comp_mean[1] + comp_mean[2]) / 3.0;
    for (int c = 0; c < 3; ++c) {
        double gain = mean_gray / std::max(comp_mean[c], eps);
        gain = std::clamp(gain, min_gain, max_gain);
        channels[c] *= gain;
    }

    cv::merge(channels, img);

    const cv::Scalar out_mean = cv::mean(img);
    const double out_gray = (out_mean[0] + out_mean[1] + out_mean[2]) / 3.0;
    if (out_gray > eps) {
        img *= mean_gray / out_gray;
    }

    img.convertTo(filtered, original.type(), 255.0);
}
}  // namespace vortex::image_filtering
#endif  // IMAGE_FILTERING__LIB__FILTERS__UNDERWATER_RED_COMPENSATION_HPP_
