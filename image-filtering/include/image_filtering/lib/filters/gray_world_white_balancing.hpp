#ifndef IMAGE_FILTERING__LIB__FILTERS__GRAY_WORLD_WHITE_BALANCING_HPP_
#define IMAGE_FILTERING__LIB__FILTERS__GRAY_WORLD_WHITE_BALANCING_HPP_

#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "abstract_filter_class.hpp"
/////////////////////////////
// Gray World White Balance
/////////////////////////////
// Assumes the average colour of the scene is gray and scales each channel to
// enforce it. Tends to remove the global colour cast (e.g. the blue/green tint
// of underwater imagery) more aggressively than SimpleWB.
//
// This is a robust variant of the classic Gray World algorithm, tailored for
// the extreme, near-uniform colour casts seen in underwater footage where the
// stock OpenCV GrayworldWB is unstable:
//   * Channel gains are computed over the WHOLE frame (no saturation cutoff),
//     so the result does not flicker as scene brightness jitters frame-to-frame
//     across a threshold.
//   * Gains are clamped to [1/max_gain, max_gain] so a degenerate frame cannot
//     collapse the image to black (or blow it out to white).
//   * Overall luminance is preserved, so correcting the colour cast does not
//     darken the image.
namespace vortex::image_filtering {
struct GrayWorldWhiteBalanceParams {
    // Upper bound on any per-channel gain (and lower bound 1/max_gain). Caps
    // how aggressively a weak channel is amplified; the main guard against the
    // black/white flicker on degenerate frames. Typical range 2-6.
    double max_gain;
};

class GrayWorldWhiteBalance : public Filter {
   public:
    explicit GrayWorldWhiteBalance(GrayWorldWhiteBalanceParams params)
        : filter_params_(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    GrayWorldWhiteBalanceParams filter_params_;
};

inline void GrayWorldWhiteBalance::apply_filter(const cv::Mat& original,
                                                cv::Mat& filtered) const {
    if (original.channels() != 3) {
        original.copyTo(filtered);
        return;
    }

    cv::Mat img;
    original.convertTo(img, CV_32F);

    // Per-channel means over the whole frame (stable frame-to-frame).
    const cv::Scalar means = cv::mean(img);
    const double mean_gray = (means[0] + means[1] + means[2]) / 3.0;

    const double eps = 1e-6;
    const double max_gain = std::max(this->filter_params_.max_gain, 1.0);
    const double min_gain = 1.0 / max_gain;

    std::vector<cv::Mat> channels;
    cv::split(img, channels);

    for (int c = 0; c < 3; ++c) {
        double gain = mean_gray / std::max(means[c], eps);
        gain = std::clamp(gain, min_gain, max_gain);
        channels[c] *= gain;
    }

    cv::merge(channels, img);

    // Preserve overall luminance: rescale so the post-correction mean matches
    // the input mean, so fixing the colour cast doesn't darken the image.
    const cv::Scalar new_means = cv::mean(img);
    const double new_mean_gray =
        (new_means[0] + new_means[1] + new_means[2]) / 3.0;
    if (new_mean_gray > eps) {
        img *= mean_gray / new_mean_gray;
    }

    img.convertTo(filtered, original.type());
}
}  // namespace vortex::image_filtering
#endif  // IMAGE_FILTERING__LIB__FILTERS__GRAY_WORLD_WHITE_BALANCING_HPP_
