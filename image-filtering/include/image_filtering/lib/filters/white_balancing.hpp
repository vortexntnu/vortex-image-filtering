#ifndef IMAGE_FILTERING__LIB__FILTERS__WHITE_BALANCING_HPP_
#define IMAGE_FILTERING__LIB__FILTERS__WHITE_BALANCING_HPP_

#include <opencv2/xphoto.hpp>
#include "abstract_filter_class.hpp"
/////////////////////////////
// White Balance
/////////////////////////////
namespace vortex::image_filtering {
struct WhiteBalanceParams {
    double contrast_percentage;
};

class WhiteBalance : public Filter {
   public:
    explicit WhiteBalance(WhiteBalanceParams params) : filter_params_(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    WhiteBalanceParams filter_params_;
};

inline void WhiteBalance::apply_filter(const cv::Mat& original,
                                       cv::Mat& filtered) const {
    double contrast_percentage = this->filter_params_.contrast_percentage;
    cv::Ptr<cv::xphoto::SimpleWB> balance = cv::xphoto::createSimpleWB();
    balance->setP(contrast_percentage);
    balance->balanceWhite(original, filtered);
}
}  // namespace vortex::image_filtering
#endif  // IMAGE_FILTERING__LIB__FILTERS__WHITE_BALANCING_HPP_
