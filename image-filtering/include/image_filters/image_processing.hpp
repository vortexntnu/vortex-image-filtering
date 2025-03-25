#ifndef IMAGE_PROCESSING_HPP
#define IMAGE_PROCESSING_HPP

#include <iostream>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>

namespace vortex::image_processing {

struct FlipParams {
    int flip_code;
};
struct UnsharpeningParams {
    int blur_size;
};

struct ErodingParams {
    int size;
};

struct DilatingParams {
    int size;
};

struct WhiteBalancingParams {
    double contrast_percentage;
};

struct EbusParams {
    int erosion_size;
    int blur_size;
    int mask_weight;
};

struct FilterParams {
    FlipParams flip;
    UnsharpeningParams unsharpening;
    ErodingParams eroding;
    DilatingParams dilating;
    WhiteBalancingParams white_balancing;
    EbusParams ebus;
};

typedef void (*FilterFunction)(const FilterParams&, const cv::Mat&, cv::Mat&);

/**
 * Reads the filter_type from the FilterParams struct
 * and calls the appropriate filter function from the filter_functions map.
 */
void apply_filter(const std::string& filter,
                  const FilterParams& params,
                  const cv::Mat& original,
                  cv::Mat& filtered);

/**
 * No filter, just copy the image
 */
void no_filter(const FilterParams& params,
               const cv::Mat& original,
               cv::Mat& modified);

/**
 * Flips the image
 */
void flip_filter(const FilterParams& params,
                 const cv::Mat& original,
                 cv::Mat& modified);

/**
 * Makes edges harder
 */
void sharpening_filter(const FilterParams& params,
                       const cv::Mat& original,
                       cv::Mat& modified);

/**
 * Makes edges harder in a smarter way
 */
void unsharpening_filter(const FilterParams& params,
                         const cv::Mat& original,
                         cv::Mat& modified);

/**
 * Expands the dark areas of the image
 */
void eroding_filter(const FilterParams& params,
                    const cv::Mat& original,
                    cv::Mat& modified);

/**
 * Expands the bright areas of the image
 */
void dilating_filter(const FilterParams& params,
                     const cv::Mat& original,
                     cv::Mat& modified);

/**
 * White Balancing Filter
 */
void white_balance_filter(const FilterParams& params,
                          const cv::Mat& original,
                          cv::Mat& filtered);

/**
 * A filter that worked well-ish in the mc-lab conditions easter 2023
 * Uses a combination of dilation and unsharpening
 */
void ebus_filter(const FilterParams& params,
                 const cv::Mat& original,
                 cv::Mat& filtered);

const static std::map<std::string, FilterFunction> filter_functions = {
    {"no_filter", no_filter},
    {"flip", flip_filter},
    {"sharpening", sharpening_filter},
    {"unsharpening", unsharpening_filter},
    {"eroding", eroding_filter},
    {"dilating", dilating_filter},
    {"white_balancing", white_balance_filter},
    {"ebus", ebus_filter}};

}  // namespace vortex::image_processing
#endif  // IMAGE_PROCESSING_HPP
