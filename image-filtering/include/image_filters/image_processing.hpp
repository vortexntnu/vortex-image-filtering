#ifndef IMAGE_PROCESSING_HPP
#define IMAGE_PROCESSING_HPP

#include <iostream>
#include <map>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>


enum class FilterType { // TODO: Add filters here
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
  Unknown
};

inline std::string to_lower(std::string s) {
  for (auto& c : s) c = static_cast<char>(std::tolower(c));
  return s;
}

inline FilterType parse_filter_type(std::string s) { // TODO: Also add filter-type here
  s = to_lower(std::move(s));
    if (s == "no_filter")      return FilterType::NoFilter;
    if (s == "flip")           return FilterType::Flip;
    if (s == "unsharpening")   return FilterType::Unsharpening;
    if (s == "erosion")        return FilterType::Erosion;
    if (s == "dilation")       return FilterType::Dilation;
    if (s == "white_balancing")return FilterType::WhiteBalancing;
    if (s == "ebus")           return FilterType::Ebus;
    if (s == "otsu")           return FilterType::Otsu;
    if (s == "overlap")        return FilterType::Overlap;
    if (s == "median_binary")  return FilterType::MedianBinary;
    if (s == "binary")         return FilterType::Binary;
    return FilterType::Unknown;
}




class Filter{
    public:
        virtual ~Filter() = default;
        virtual void  apply_filter(const cv::Mat& original, cv::Mat& filtered) const = 0;
        
    protected:
        Filter() = default;
};


/////////////////////////////
// No filter
/////////////////////////////

struct NoFilterParams{};

class NoFilter: public Filter{
    public:
        explicit NoFilter() = default;// No parameters to set
        void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override{ original.copyTo(filtered); };
};


////////////////////////////
// Unsharpening
/////////////////////////////

struct UnsharpeningParams{ 
    int blur_size;
};


class Unsharpening: public Filter{
    public:
        explicit Unsharpening(UnsharpeningParams params): filter_params(params) {}
        void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override;
    private:
        UnsharpeningParams filter_params;
};



/////////////////////////////
// Sharpening
/////////////////////////////

struct FlipParams{
    int flip_code;
};

class Flip: public Filter{
    public:
        explicit Flip(FlipParams params): filter_params(params) {}
        void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override;
    private:
        FlipParams filter_params;
};






/////////////////////////////
// Sharpening
/////////////////////////////

struct SharpeningParams{};

class Sharpening: public Filter{
public:
    explicit Sharpening(SharpeningParams params): filter_params(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override;
private:
    SharpeningParams filter_params;
};
/////////////////////////////
// Erosion
/////////////////////////////

struct ErosionParams{
    int kernel_size;   // odd > 1
};

class Erosion: public Filter{
public:
    explicit Erosion(ErosionParams params): filter_params(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override;
private:
    ErosionParams filter_params;
};

/////////////////////////////
// Dilation
/////////////////////////////

struct DilationParams{
    int kernel_size = 3;
};

class Dilation: public Filter{
public:
    explicit Dilation(DilationParams params): filter_params(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override;
private:
    DilationParams filter_params;
};

/////////////////////////////
// White Balance
/////////////////////////////

struct WhiteBalanceParams{
    double contrast_percentage;
};

class WhiteBalance: public Filter{
public:
    explicit WhiteBalance(WhiteBalanceParams params): filter_params(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override;
private:
    WhiteBalanceParams filter_params;
};

/////////////////////////////
// Ebus (dilation + unsharpening combo)
/////////////////////////////

struct EbusParams{
    int erosion_size;
    int blur_size;
    int mask_weight;
};

class Ebus: public Filter{
public:
    explicit Ebus(EbusParams params): filter_params(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override;
private:
    EbusParams filter_params;
};

/////////////////////////////
// Otsu Segmentation
/////////////////////////////

struct OtsuSegmentationParams{
    bool gamma_auto_correction;
    double gamma_auto_correction_weight;
    bool otsu_segmentation;
    double gsc_weight_r;
    double gsc_weight_g;
    double gsc_weight_b;
    int erosion_size;
    int dilation_size;
};

class OtsuSegmentation: public Filter{
public:
    explicit OtsuSegmentation(OtsuSegmentationParams params): filter_params(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& output) const override;
private:
    OtsuSegmentationParams filter_params;
};

/////////////////////////////
// Overlap (blend/composite)
/////////////////////////////

struct OverlapParams{
    double percentage_threshold;
};

class Overlap: public Filter{
public:
    explicit Overlap(OverlapParams params): filter_params(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override;
private:
    OverlapParams filter_params;
};

/////////////////////////////
// Median + Binary
/////////////////////////////

struct MedianBinaryParams{
    int kernel_size;
    int threshold;
    bool invert;
};

class MedianBinary: public Filter{
public:
    explicit MedianBinary(MedianBinaryParams params): filter_params(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override;
private:
    MedianBinaryParams filter_params;
};

/////////////////////////////
// Binary Threshold
/////////////////////////////

struct BinaryThresholdParams{
    double threshold;
    double maxval;
    bool invert;
};

class BinaryThreshold: public Filter{
public:
    explicit BinaryThreshold(BinaryThresholdParams params): filter_params(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override;
private:
    BinaryThresholdParams filter_params;
};






/////////////////////////////
// Template
/////////////////////////////

// TODO: add this structure for your filter

// Template:
struct ExampleParams{ // Add filter variables here
    int example_variable;
    std::string example_string;
};

class Example: public Filter{
    public:
        explicit Example(ExampleParams params): filter_params(params) {}
        void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override; // This is the filter itself
    private:
        ExampleParams filter_params;
};













#endif  // IMAGE_PROCESSING_HPP



