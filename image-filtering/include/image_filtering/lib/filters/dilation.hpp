#ifndef LIB__FILTERS__DILATION_HPP_
#define LIB__FILTERS__DILATION_HPP_

#include "abstract_filter_class.hpp"
#include "image_filtering/lib/utilities.hpp"

/////////////////////////////
// Dilation
/////////////////////////////

namespace vortex::image_filtering {

struct DilationParams {
    int kernel_size = 3;
};

class Dilation : public Filter {
   public:
    explicit Dilation(DilationParams params) : filter_params_(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    DilationParams filter_params_;
};

inline void Dilation::apply_filter(const cv::Mat& original,
                                   cv::Mat& filtered) const {
    apply_dilation(original, filtered, this->filter_params_.kernel_size,
                   cv::MORPH_RECT);
}
}  // namespace vortex::image_filtering
#endif  // LIB__FILTERS__DILATION_HPP_
