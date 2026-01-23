
#ifndef LIB__filters__DILATION_HPP_
#define LIB__filters__DILATION_HPP_

#include "abstract_filter_class.hpp"
#include "lib/utilities.hpp"

/////////////////////////////
// Dilation
/////////////////////////////

struct DilationParams {
    int kernel_size = 3;
};

class Dilation : public Filter {
   public:
    explicit Dilation(DilationParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    DilationParams filter_params;
};

inline void Dilation::apply_filter(const cv::Mat& original,
                                   cv::Mat& filtered) const {
    apply_dilation(original, filtered, this->filter_params.kernel_size,
                   cv::MORPH_RECT);
}

#endif  // LIB__filters__DILATION_HPP_
