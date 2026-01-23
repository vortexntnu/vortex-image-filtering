

#ifndef LIB__FILTERS__BINARY_THRESHOLD_HPP_
#define LIB__FILTERS__BINARY_THRESHOLD_HPP_

#include "abstract_filter_class.hpp"

/////////////////////////////
// Binary Threshold
/////////////////////////////

struct BinaryThresholdParams {
    double threshold;
    double maxval;
    bool invert;
};

class BinaryThreshold : public Filter {
   public:
    explicit BinaryThreshold(BinaryThresholdParams params)
        : filter_params(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    BinaryThresholdParams filter_params;
};

inline void BinaryThreshold::apply_filter(const cv::Mat& original,
                                          cv::Mat& filtered) const {
    CV_Assert(!original.empty());

    const double thresh = this->filter_params.threshold;
    const double maxval = this->filter_params.maxval;
    const bool invert = this->filter_params.invert;

    // 1) Ensure single-channel
    cv::Mat gray;
    if (original.channels() == 1) {
        gray = original;
    } else {
        cv::cvtColor(original, gray, cv::COLOR_BGR2GRAY);
    }

    // Standardize to 8-bit (safe for thresholding)
    cv::Mat src8;
    if (gray.depth() != CV_8U) {
        // Adjust scaling here if grayscale is not already 0–255
        gray.convertTo(src8, CV_8U);
    } else {
        src8 = gray;
    }
    // Apply fixed threshold
    const int type = invert ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
    cv::threshold(src8, filtered, thresh, maxval, type);
}

#endif  // LIB__FILTERS__BINARY_THRESHOLD_HPP_
