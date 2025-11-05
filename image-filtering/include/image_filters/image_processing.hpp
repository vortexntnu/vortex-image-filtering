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


// struct FilterParams {
//     FlipParams flip;
//     UnsharpeningParams unsharpening;
//     ErodingParams eroding;
//     DilatingParams dilating;
//     WhiteBalancingParams white_balancing;
//     EbusParams ebus;
//     OtsuParams otsu;
//     OverlapParams overlap;
//     MedianBinaryParams median_binary;
//     BinaryParams binary;
// };

struct NoFilterStruct{};

struct FlipFiltersTructer {
    int flip_code;
};

/**
 * Reads the filter_type from the FilterParams struct
 * and calls the appropriate filter function from the filter_functions map.
 */

virtual class Filter{
    virtual void apply_filter(const cv::Mat& original, cv::Mat& filtered);
};

/**
 * No filter, just copy the image
 */
class NoFilter(const NoFilterStruct& args): public Filter{
    void no_filter(const FilterParams& params,
                   const cv::Mat& original,
                   cv::Mat& modified);
    private:
        int flip_code;
};

virtual class FilterClass(FilterStuct args);
/**
 * Flips the image
 */
class Flip (Filter) {
public:
    int flip_code;

    
};

/**
 * Makes edges harder
 */
class Sharpening {
    void sharpening_filter(const FilterParams& params,
                           const cv::Mat& original,
                           cv::Mat& modified);
};

/**
 * Makes edges harder in a smarter way
 */
class Unsharpening {
public:
    int blur_size;
    void unsharpening_filter(const FilterParams& params,
                             const cv::Mat& original,
                             cv::Mat& modified);
};

/**
 * Expands the dark areas of the image
 */
class Erosion {
public:
    int size;
    void erosion_filter(const FilterParams& params,
                        const cv::Mat& original,
                        cv::Mat& modified);
};

/**
 * Expands the bright areas of the image
 */
class Dilation {
public:
    int size;
    void dilation_filter(const FilterParams& params,
                         const cv::Mat& original,
                         cv::Mat& modified);
};

/**
 * White Balancing Filter
 */
class WhiteBalance {
public:
    double contrast_percentage;
    void white_balance_filter(const FilterParams& params,
                              const cv::Mat& original,
                              cv::Mat& filtered);
};

/**
 * A filter that worked well-ish in the mc-lab conditions easter 2023
 * Uses a combination of dilation and unsharpening
 */
class Ebus {
public:
    int erosion_size;
    int blur_size;
    int mask_weight;
    void ebus_filter(const FilterParams& params,
                     const cv::Mat& original,
                     cv::Mat& filtered);
};

/**
 * A filter based on Otsu's method
 */
class OtsuSegmentation {
public:
    bool gamma_auto_correction;
    double gamma_auto_correction_weight;
    bool otsu_segmentation;
    double gsc_weight_r;
    double gsc_weight_g;
    double gsc_weight_b;
    int erosion_size;
    int dilation_size;

    void otsu_segmentation_filter(const FilterParams& params,
                                  const cv::Mat& original,
                                  cv::Mat& output);
};

class Overlap {
public:
    // Thomas' masterpiece
    double percentage_threshold;

    void overlap_filter(const FilterParams& filter_params,
                        const cv::Mat &original, 
                        cv::Mat &filtered);
};

class MedianBinary {
public:
    int kernel_size;
    int threshold;
    bool invert;

    void median_binary_filter(const FilterParams& filter_params,
                              const cv::Mat& original,
                              cv::Mat& filtered);
};

class BinaryThreshold {
public:
    double threshold;
    double maxval;
    bool invert;

    void binary_threshold(const FilterParams& filter_params,
                          const cv::Mat& original,
                          cv::Mat& filtered);
};


const static std::map<std::string, FilterFunction> filter_functions = {
    {"no_filter", no_filter},
    {"flip", flip_filter},
    {"sharpening", sharpening_filter},
    {"unsharpening", unsharpening_filter},
    {"erosion", erosion_filter},
    {"dilation", dilation_filter},
    {"white_balancing", white_balance_filter},
    {"ebus", ebus_filter},
    {"otsu", otsu_segmentation_filter},
    {"overlap", Overlap::overlap_filter}, // This was done by the one and only Thomas
    {"median_binary", median_binary_filter},
    {"binary", binary_threshold},
};

#endif  // IMAGE_PROCESSING_HPP



