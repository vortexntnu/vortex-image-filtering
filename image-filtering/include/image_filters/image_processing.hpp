#ifndef IMAGE_FILTERS__IMAGE_PROCESSING_HPP_
#define IMAGE_FILTERS__IMAGE_PROCESSING_HPP_


#include "filters/abstract_filter_class.hpp"

/////////////////////////////
// No filter
/////////////////////////////

struct NoFilterParams {};

class NoFilter : public Filter {
   public:
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override {
        original.copyTo(filtered);
    };
};

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

/////////////////////////////
// Sharpening
/////////////////////////////

struct FlipParams {
    int flip_code;
};

class Flip : public Filter {
   public:
    explicit Flip(FlipParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    FlipParams filter_params;
};

/////////////////////////////
// Sharpening
/////////////////////////////

struct SharpeningParams {};

class Sharpening : public Filter {
   public:
    explicit Sharpening(SharpeningParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    SharpeningParams filter_params;
};
/////////////////////////////
// Erosion
/////////////////////////////

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

/////////////////////////////
// White Balance
/////////////////////////////

struct WhiteBalanceParams {
    double contrast_percentage;
};

class WhiteBalance : public Filter {
   public:
    explicit WhiteBalance(WhiteBalanceParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    WhiteBalanceParams filter_params;
};

/////////////////////////////
// Ebus (dilation + unsharpening combo)
/////////////////////////////

struct EbusParams {
    int erosion_size;
    int blur_size;
    int mask_weight;
};

class Ebus : public Filter {
   public:
    explicit Ebus(EbusParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    EbusParams filter_params;
};

/////////////////////////////
// Otsu Segmentation
/////////////////////////////

struct OtsuSegmentationParams {
    bool gamma_auto_correction;
    double gamma_auto_correction_weight;
    bool otsu_segmentation;
    double gsc_weight_r;
    double gsc_weight_g;
    double gsc_weight_b;
    int erosion_size;
    int dilation_size;
};

class OtsuSegmentation : public Filter {
   public:
    explicit OtsuSegmentation(OtsuSegmentationParams params)
        : filter_params(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& output) const override;

   private:
    OtsuSegmentationParams filter_params;
};

/////////////////////////////
// Overlap (blend/composite)
/////////////////////////////

struct OverlapParams {
    double percentage_threshold;  // 0..100 (percent)
};

class Overlap : public Filter {
   public:
    explicit Overlap(OverlapParams params)
        : filter_params(params), has_prev(false) {}

    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    OverlapParams filter_params;

    // Cached previous mono8 frame
    mutable cv::Mat prev;
    mutable bool has_prev;
};

/////////////////////////////
// Median + Binary
/////////////////////////////

struct MedianBinaryParams {
    int kernel_size;
    int threshold;
    bool invert;
};

class MedianBinary : public Filter {
   public:
    explicit MedianBinary(MedianBinaryParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    MedianBinaryParams filter_params;
};

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



#endif  // IMAGE_FILTERS__IMAGE_PROCESSING_HPP_
