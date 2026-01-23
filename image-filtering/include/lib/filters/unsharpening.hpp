

#ifndef LIB__FILTERS__UNSHARPENING_HPP_
#define LIB__FILTERS__UNSHARPENING_HPP_

#include "abstract_filter_class.hpp"

////////////////////////////
// Unsharpening
/////////////////////////////

struct UnsharpeningParams {
    int blur_size;
};

class Unsharpening : public Filter {
   public:
    explicit Unsharpening(UnsharpeningParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    UnsharpeningParams filter_params;
};

inline void Unsharpening::apply_filter(const cv::Mat& original,
                                       cv::Mat& filtered) const {
    int blur_size = this->filter_params.blur_size;
    // Create a blurred version of the image
    cv::Mat blurred;
    GaussianBlur(original, blurred,
                 cv::Size(2 * blur_size + 1, 2 * blur_size + 1), 0);

    // Compute the unsharp mask
    cv::Mat mask = original - blurred;
    cv::Mat unsharp;

    addWeighted(original, 1, mask, 3, 0, filtered);
}

#endif  // LIB__FILTERS__UNSHARPENING_HPP_
