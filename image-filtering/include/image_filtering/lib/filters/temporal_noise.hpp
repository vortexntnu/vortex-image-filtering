#ifndef IMAGE_FILTERING__LIB__FILTERS__SONAR_NOISE_HPP_
#define IMAGE_FILTERING__LIB__FILTERS__SONAR_NOIse_HPP_

#include "abstract_filter_class.hpp"
#include "image_filtering/lib/utilities.hpp"

/////////////////////////////
// Sonar noise
/////////////////////////////
namespace vortex::image_filtering {
struct TemporalNoiseParams {
    int dontknow;
};

class TemporalNoise : public Filter {
   public:
    explicit TemporalNoise(TemporalNoiseParams params) : filter_params_(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    TemporalNoiseParams filter_params_;
};

inline void TemporalNoise::apply_filter(const cv::Mat& original, cv::Mat& filtered) const {
    // TODO: finish him
    cv::flip(original, filtered, 1);
}
}  // namespace vortex::image_filtering
#endif  // IMAGE_FILTERING__LIB__FILTERS__SONAR_NOISE_HPP_
