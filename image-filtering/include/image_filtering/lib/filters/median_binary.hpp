#ifndef LIB__FILTERS__MEDIAN_BINARY_HPP_
#define LIB__FILTERS__MEDIAN_BINARY_HPP_

#include "abstract_filter_class.hpp"
#include "image_filtering/lib/utilities.hpp"
/////////////////////////////
// Median + Binary
/////////////////////////////
namespace vortex::image_filtering {
struct MedianBinaryParams {
    int kernel_size;
    int threshold;
    bool invert;
};

class MedianBinary : public Filter {
   public:
    explicit MedianBinary(MedianBinaryParams params) : filter_params_(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    MedianBinaryParams filter_params_;
};

inline void MedianBinary::apply_filter(const cv::Mat& original,
                                       cv::Mat& filtered) const {
    apply_median(original, filtered, this->filter_params_.kernel_size);
    apply_fixed_threshold(filtered, filtered, this->filter_params_.threshold,
                          this->filter_params_.invert);
}
}  // namespace vortex::image_filtering
#endif  // LIB__FILTERS__MEDIAN_BINARY_HPP_
