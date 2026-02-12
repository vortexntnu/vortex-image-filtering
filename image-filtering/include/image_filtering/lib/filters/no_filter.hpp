#ifndef LIB__FILTERS__NO_FILTER_HPP_
#define LIB__FILTERS__NO_FILTER_HPP_

#include "abstract_filter_class.hpp"

/////////////////////////////
// No filter
/////////////////////////////
namespace vortex::image_filtering {
struct NoFilterParams {};

class NoFilter : public Filter {
   public:
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override {
        original.copyTo(filtered);
    };
};
}  // namespace vortex::image_filtering
#endif  // LIB__FILTERS__NO_FILTER_HPP_
