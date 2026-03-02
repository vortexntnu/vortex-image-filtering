#ifndef IMAGE_FILTERING__LIB__FILTERS__TEMPORAL_NOISE_HPP_
#define IMAGE_FILTERING__LIB__FILTERS__TEMPORAL_NOISE_HPP_

#include "abstract_filter_class.hpp"
#include "image_filtering/lib/utilities.hpp"

/////////////////////////////
// Sonar noise
/////////////////////////////
namespace vortex::image_filtering {
struct TemporalNoiseParams {
    int median_kernel_size;
    double power_law_weight;

    int canny_low;
    int canny_high;
    int edge_protection_radius;

    int erotion_size;
    int dilation_size;
};

class TemporalNoise : public Filter {
   public:
    explicit TemporalNoise(TemporalNoiseParams params)
        : filter_params_(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    TemporalNoiseParams filter_params_;
    mutable cv::Mat previous_;
    mutable bool has_prev_{false};
};

inline void TemporalNoise::apply_filter(const cv::Mat& original,
                                        cv::Mat& filtered) const {
    const double power_law_weight = filter_params_.power_law_weight;
    const int erosion_size = filter_params_.erotion_size;
    const int dilation_size = filter_params_.dilation_size;
    const int protect_radius = filter_params_.edge_protection_radius;
    const int canny_low = filter_params_.canny_low;
    const int canny_high = filter_params_.canny_high;
    const int median_kernel_size = filter_params_.median_kernel_size;

    cv::Mat temp;
    original.copyTo(temp);

    apply_median(temp, temp, median_kernel_size);

    apply_auto_gamma(temp, power_law_weight);

    if (!has_prev_) {
        temp.copyTo(previous_);
        temp.copyTo(filtered);
        has_prev_ = true;
    } else {
        cv::addWeighted(temp, 0.5, previous_, 0.5, 0.0, filtered);
        temp.copyTo(previous_);
    }

    cv::Mat edges;
    cv::Canny(filtered, edges, canny_low, canny_high);

    // Thicken mask a bit so we protect the whole line, not just 1px edge.
    if (protect_radius > 0) {
        cv::Mat k = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(2 * protect_radius + 1, 2 * protect_radius + 1));
        cv::dilate(edges, edges, k);
    }

    // Invert mask: where NOT edges => safe to morph aggressively.
    cv::Mat not_edges;
    cv::bitwise_not(edges, not_edges);

    // Morphing only outside the protected areas
    cv::Mat morphed = filtered.clone();

    apply_erosion(morphed, morphed, erosion_size);
    apply_dilation(morphed, morphed, dilation_size);

    morphed.copyTo(filtered, not_edges);
}
}  // namespace vortex::image_filtering
#endif  // IMAGE_FILTERING__LIB__FILTERS__TEMPORAL_NOISE_HPP_
