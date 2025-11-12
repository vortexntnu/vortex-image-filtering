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





struct FlipFilterParams {
    int flip_code;
};



class Filter{
    public:
        virtual ~Filter() = default;
        virtual void  apply_filter(const cv::Mat& original, cv::Mat& filtered) const = 0;
        
    protected:
        Filter() = default;
};

/////////////////////////////

struct NoFilterParams{};

 // (const NoFilterStruct& args)
class NoFilter: public Filter{
    public:
        explicit NoFilter() = default;// No parameters to set
        void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override{ original.copyTo(filtered); };
};

////////////////////////////

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




////////////////////////////////////

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



