#ifndef IMAGE_FILTERING__LIB__FILTERS__WHITE_PATCH_HPP_
#define IMAGE_FILTERING__LIB__FILTERS__WHITE_PATCH_HPP_

#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "abstract_filter_class.hpp"
/////////////////////////////
// White Patch / Max-RGB
/////////////////////////////
// White-Patch Retinex colour constancy: assumes the brightest value in each
// channel corresponds to a white surface, and scales each channel so that white
// point maps to full intensity. Instead of the literal per-channel maximum
// (very noise sensitive) a high percentile is used as a robust "white"
// reference.
namespace vortex::image_filtering {
struct WhitePatchParams {
    // Percentile [0, 1] of each channel taken as its white reference. 1.0 is
    // the true maximum (noise sensitive); 0.95-0.99 is more robust.
    double percentile;
    // Clamp on per-channel gain ([1/max_gain, max_gain]) for stability.
    double max_gain;
};

class WhitePatch : public Filter {
   public:
    explicit WhitePatch(WhitePatchParams params) : filter_params_(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    WhitePatchParams filter_params_;
};

inline void WhitePatch::apply_filter(const cv::Mat& original,
                                     cv::Mat& filtered) const {
    if (original.channels() != 3) {
        original.copyTo(filtered);
        return;
    }

    // Work on an 8-bit copy for the histogram-based percentile.
    cv::Mat img8u = original;
    if (original.depth() != CV_8U) {
        cv::normalize(original, img8u, 0, 255, cv::NORM_MINMAX);
        img8u.convertTo(img8u, CV_8U);
    }

    std::vector<cv::Mat> channels8u;
    cv::split(img8u, channels8u);

    // Value below which `percentile` of a channel's pixels lie.
    auto channel_percentile = [](const cv::Mat& ch, double pct) -> double {
        int hist_size = 256;
        float range[] = {0.0F, 256.0F};
        const float* hist_range = range;
        cv::Mat hist;
        cv::calcHist(&ch, 1, nullptr, cv::Mat(), hist, 1, &hist_size,
                     &hist_range);
        const double target = pct * ch.total();
        double cumulative = 0.0;
        for (int i = 0; i < hist_size; ++i) {
            cumulative += hist.at<float>(i);
            if (cumulative >= target) {
                return static_cast<double>(i);
            }
        }
        return 255.0;
    };

    const double pct = std::clamp(this->filter_params_.percentile, 0.0, 1.0);
    const double eps = 1e-6;
    const double max_gain = std::max(this->filter_params_.max_gain, 1.0);
    const double min_gain = 1.0 / max_gain;

    cv::Mat img;
    original.convertTo(img, CV_32F);
    std::vector<cv::Mat> channels;
    cv::split(img, channels);

    // Anchor each channel's white reference to full scale (255).
    for (int c = 0; c < 3; ++c) {
        double white = channel_percentile(channels8u[c], pct);
        double gain = 255.0 / std::max(white, eps);
        gain = std::clamp(gain, min_gain, max_gain);
        channels[c] *= gain;
    }

    cv::merge(channels, img);
    img.convertTo(filtered, original.type());
}
}  // namespace vortex::image_filtering
#endif  // IMAGE_FILTERING__LIB__FILTERS__WHITE_PATCH_HPP_
