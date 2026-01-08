#ifndef IMAGE_FILTERS__IMAGE_PROCESSING_HPP_
#define IMAGE_FILTERS__IMAGE_PROCESSING_HPP_

#include <iostream>
#include <map>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>
#include <string>
#include <utility>

enum class FilterType {  // TODO(Vortex): Add filters here
    NoFilter,
    Flip,
    Unsharpening,
    Erosion,
    Dilation,
    WhiteBalancing,
    Ebus,
    Otsu,
    Overlap,
    MedianBinary,
    Binary,

    Example,
    Unknown
};

static constexpr std::pair<std::string_view, FilterType> kFilterMap[] = {
    {"no_filter", FilterType::NoFilter},
    {"flip", FilterType::Flip},
    {"unsharpening", FilterType::Unsharpening},
    {"erosion", FilterType::Erosion},
    {"dilation", FilterType::Dilation},
    {"white_balancing", FilterType::WhiteBalancing},
    {"ebus", FilterType::Ebus},
    {"otsu", FilterType::Otsu},
    {"overlap", FilterType::Overlap},
    {"median_binary", FilterType::MedianBinary},
    {"binary", FilterType::Binary},

    // TODO(Vortex): Also add your filter here
    {"example", FilterType::Example},
    {"unknown", FilterType::Unknown}};

inline std::string to_lower(std::string s) {
    for (char& ch : s) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return s;
}

inline FilterType parse_filter_type(std::string s) {
    s = to_lower(std::move(s));

    for (auto [name, type] : kFilterMap) {
        if (s == name)
            return type;
    }
    std::cout << "\033[33m No string connected to that filter type: '" << s
              << "'. This might be misspelling or you need to add the filter "
                 "type to kFilterMap in image_processing.hpp\033[0m";
    return FilterType::Unknown;
}

inline std::string_view filtertype_to_string(FilterType t) {
    for (auto [name, type] : kFilterMap) {
        if (t == type)
            return name;
    }
    std::cout << "\033[33m No string connected to your filter type. To fix "
                 "this add the string and filter type to kFilterMap\033[0m";
    return "unknown";
}

class Filter {
   public:
    virtual ~Filter() = default;
    virtual void apply_filter(const cv::Mat& original,
                              cv::Mat& filtered) const = 0;

   protected:
    Filter() = default;
};

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

// TODO(Vortex): add this structure for your filter

/////////////////////////////
// Example
/////////////////////////////

// Example:
struct ExampleParams {  // Add filter variables here
    int example_int;
    std::string example_string;
};

class Example : public Filter {
   public:
    explicit Example(ExampleParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& filtered)
        const override;  // This is the filter itself
   private:
    ExampleParams filter_params;
};

#endif  // IMAGE_FILTERS__IMAGE_PROCESSING_HPP_
