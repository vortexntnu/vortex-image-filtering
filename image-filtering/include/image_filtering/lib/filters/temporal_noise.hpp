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
    double blur_sigma;
    double gamma_auto_correction_weight;

    int erotion_size;
    int dilation_size;

};

class TemporalNoise : public Filter {
   public:
    explicit TemporalNoise(TemporalNoiseParams params) : filter_params_(params), has_prev_(false) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    TemporalNoiseParams filter_params_;
    mutable cv::Mat previous_;
    mutable bool has_prev_;
};

inline void TemporalNoise::apply_filter(const cv::Mat& original, cv::Mat& filtered) const {

    double sigma = filter_params_.blur_sigma;
    double auto_gama_weight = filter_params_.gamma_auto_correction_weight;
    int erotion_size = filter_params_.erotion_size;
    int dilation_size = filter_params_.dilation_size;

    cv::Mat temp;

    cv::GaussianBlur(original, temp, cv::Size(0,0), sigma, sigma, cv::BORDER_REPLICATE);

    apply_auto_gamma(temp, auto_gama_weight);
    
    if (!has_prev_){
        has_prev_ = true;
        temp.copyTo(previous_);
        temp.copyTo(filtered);
    } else {
        cv::addWeighted(temp, 0.5, previous_, 0.5, 0.0, filtered);
        previous_ = temp.clone(); // update for next call
    }


    apply_erosion(filtered, temp, erotion_size);
    apply_dilation(temp, filtered, dilation_size);
    
}
}  // namespace vortex::image_filtering
#endif  // IMAGE_FILTERING__LIB__FILTERS__SONAR_NOISE_HPP_
