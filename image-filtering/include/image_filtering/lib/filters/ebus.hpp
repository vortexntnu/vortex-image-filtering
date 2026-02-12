#ifndef LIB__FILTERS__EBUS_HPP_
#define LIB__FILTERS__EBUS_HPP_

#include "abstract_filter_class.hpp"
#include "image_filtering/lib/utilities.hpp"

/////////////////////////////
// Ebus (dilation + unsharpening combo)
/////////////////////////////

namespace vortex::image_filtering {
struct EbusParams {
    int erosion_size;
    int blur_size;
    int mask_weight;
};
class Ebus : public Filter {
   public:
    explicit Ebus(EbusParams params) : filter_params_(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    EbusParams filter_params_;
};

inline void Ebus::apply_filter(const cv::Mat& original,
                               cv::Mat& filtered) const {
    int blur_size = this->filter_params_.blur_size;
    int mask_weight = this->filter_params_.mask_weight;
    int erosion_size = this->filter_params_.erosion_size;
    // Erode image to make blacks more black
    cv::Mat eroded;

    apply_erosion(original, eroded, erosion_size);

    // Make an unsharp mask from original image
    cv::Mat blurred;
    GaussianBlur(original, blurred,
                 cv::Size(2 * blur_size + 1, 2 * blur_size + 1), 0);

    // Compute the unsharp mask
    cv::Mat mask = original - blurred;
    cv::Mat unsharp;

    // Add mask to the eroded image instead of the original
    // Higher weight of mask will create a sharper but more noisy image
    addWeighted(eroded, 1, mask, mask_weight, 0, filtered);
}
}  // namespace vortex::image_filtering

#endif  // LIB__FILTERS__EBUS_HPP_
