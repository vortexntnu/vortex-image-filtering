
#ifndef LIB__FILTERS__EROSION_HPP_
#define LIB__FILTERS__EROSION_HPP_

#include "abstract_filter_class.hpp"
#include "lib/utilities.hpp"

/////////////////////////////
// Erosion
/////////////////////////////
namespace vortex::image_filtering {
struct ErosionParams {
    int kernel_size;  // odd > 1
};

class Erosion : public Filter {
   public:
    explicit Erosion(ErosionParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    ErosionParams filter_params;
};

inline void Erosion::apply_filter(const cv::Mat& original,
                                  cv::Mat& filtered) const {
    apply_erosion(original, filtered, this->filter_params.kernel_size,
                  cv::MORPH_RECT);
}
}

#endif  // LIB__FILTERS__EROSION_HPP_
