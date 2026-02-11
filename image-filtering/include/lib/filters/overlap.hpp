#ifndef LIB__FILTERS__OVERLAP_HPP_
#define LIB__FILTERS__OVERLAP_HPP_

#include "abstract_filter_class.hpp"
#include "opencv2/imgproc.hpp"

/////////////////////////////
// Overlap (blend/composite)
/////////////////////////////
namespace vortex::image_filtering {
struct OverlapParams {
    double percentage_threshold;  // 0..100 (percent)
};

class Overlap : public Filter {
   public:
    explicit Overlap(OverlapParams params)
        : filter_params(params), has_prev(false) {}

    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    OverlapParams filter_params;

    // Cached previous mono8 frame
    mutable cv::Mat prev;
    mutable bool has_prev;
};

inline void Overlap::apply_filter(const cv::Mat& original,
                                  cv::Mat& filtered) const {
    // mono8 only
    CV_Assert(!original.empty());
    CV_Assert(original.type() == CV_8UC1);

    double thr_percent = filter_params.percentage_threshold;

    // First frame (or size/type change): passthrough + cache
    if (!has_prev || prev.empty() || prev.size() != original.size() ||
        prev.type() != original.type()) {
        original.copyTo(filtered);
        prev = original.clone();
        has_prev = true;
        return;
    }

    // Absolute difference (still mono8)
    cv::Mat diff8u;
    cv::absdiff(original, prev, diff8u);

    // Convert to percent of 8-bit range (0..100)
    cv::Mat percent32f;
    diff8u.convertTo(percent32f, CV_32F, 100.0 / 255.0);

    // Build mask where change > threshold
    cv::Mat mask8u;
    cv::threshold(percent32f, mask8u, thr_percent, 255.0, cv::THRESH_BINARY);
    mask8u.convertTo(mask8u, CV_8U);

    // Output: same as original except zero-out "changed" pixels
    filtered = original.clone();
    filtered.setTo(0, mask8u);

    // Update history (write to cached previous)
    prev = original.clone();
}
}
#endif  // LIB__FILTERS__OVERLAP_HPP_
