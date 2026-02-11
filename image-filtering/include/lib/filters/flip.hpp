#ifndef LIB__FILTERS__FLIP_HPP_
#define LIB__FILTERS__FLIP_HPP_

#include "abstract_filter_class.hpp"

/////////////////////////////
// Flip
/////////////////////////////
namespace vortex::image_filtering {
struct FlipParams {
    int flip_code;
};

class Flip : public Filter {
   public:
    explicit Flip(FlipParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    FlipParams filter_params;
};

inline void Flip::apply_filter(const cv::Mat& original,
                               cv::Mat& filtered) const {
    int flip_code =
        this->filter_params.flip_code;  // 0: x-axis, 1: y-axis, -1: both
    cv::flip(original, filtered, flip_code);
}
}
#endif  // LIB__FILTERS__FLIP_HPP_
