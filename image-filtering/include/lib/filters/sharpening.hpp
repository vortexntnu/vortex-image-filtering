#ifndef LIB__FILTERS__SHARPENING_HPP_
#define LIB__FILTERS__SHARPENING_HPP_

#include "abstract_filter_class.hpp"
#include "opencv2/imgproc.hpp"

/////////////////////////////
// Sharpening
/////////////////////////////
namespace vortex::image_filtering {
struct SharpeningParams {};

class Sharpening : public Filter {
   public:
    explicit Sharpening(SharpeningParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    SharpeningParams filter_params;
};

inline void Sharpening::apply_filter(const cv::Mat& original,
                                     cv::Mat& filtered) const {
    // Sharpen image
    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    cv::filter2D(original, filtered, -1, kernel);
}
}  // namespace vortex::image_filtering
#endif  // LIB__FILTERS__SHARPENING_HPP_
