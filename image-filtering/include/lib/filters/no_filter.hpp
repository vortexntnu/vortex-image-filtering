
#ifndef IMAGE_FILTERS__filters__NO_FILTER_HPP_
#define IMAGE_FILTERS__filters__NO_FILTER_HPP_

#include "abstract_filter_class.hpp"

/////////////////////////////
// No filter
/////////////////////////////

struct NoFilterParams {};

class NoFilter : public Filter {
   public:
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override {
        original.copyTo(filtered);
    };
};

#endif  // IMAGE_FILTERS__filters__NO_FILTER_HPP_
