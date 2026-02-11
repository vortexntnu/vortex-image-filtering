#ifndef LIB__FILTERS__MEDIAN_BINARY_HPP_
#define LIB__FILTERS__MEDIAN_BINARY_HPP_

#include "abstract_filter_class.hpp"
#include "lib/utilities.hpp"
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
    explicit MedianBinary(MedianBinaryParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    MedianBinaryParams filter_params;
};

inline void MedianBinary::apply_filter(const cv::Mat& original,
                                       cv::Mat& filtered) const {
    apply_median(original, filtered, this->filter_params.kernel_size);
    apply_fixed_threshold(filtered, filtered, this->filter_params.threshold,
                          this->filter_params.invert);
}
}  // namespace vortex::image_filtering
#endif  // LIB__FILTERS__MEDIAN_BINARY_HPP_
