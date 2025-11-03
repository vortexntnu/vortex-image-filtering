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

struct OtsuParams {
    bool gamma_auto_correction;
    double gamma_auto_correction_weight;
    bool otsu_segmentation;
    double gsc_weight_r;
    double gsc_weight_g;
    double gsc_weight_b;
    int erosion_size;
    int dilation_size;
};

// Thomas' masterpiece
struct OverlapParams{
    double percentage_threshold;
};
struct MedianBinaryParams{
    int kernel_size;
    int threshold;
    bool invert;
};

struct BinaryParams{
    double threshold;
    double maxval;
    bool invert;
};

struct FilterParams {
    FlipParams flip;
    UnsharpeningParams unsharpening;
    ErodingParams eroding;
    DilatingParams dilating;
    WhiteBalancingParams white_balancing;
    EbusParams ebus;
    OtsuParams otsu;
    OverlapParams overlap;
    MedianBinaryParams median_binary;
    BinaryParams binary;
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
void erosion_filter(const FilterParams& params,
                    const cv::Mat& original,
                    cv::Mat& modified);

/**
 * Expands the bright areas of the image
 */
void dilation_filter(const FilterParams& params,
                     const cv::Mat& original,
                     cv::Mat& modified);

/**
 * White Balancing Filter
 */
void white_balance_filter(const FilterParams& params,
                          const cv::Mat& original,
                          cv::Mat& filtered);

/**￼
 * A filter that worked well-ish in the mc-lab conditions easter 2023
 * Uses a combination of dilation and unsharpening
 */
void ebus_filter(const FilterParams& params,
                 const cv::Mat& original,
                 cv::Mat& filtered);

/**
 * A filter based on Otsu's method
 */
void otsu_segmentation_filter(const FilterParams& params,
                              const cv::Mat& original,
                              cv::Mat& output);


void overlap_filter(const FilterParams& filter_params,
                          const cv::Mat &original, 
                          cv::Mat &filtered);


void median_binary_filter(const FilterParams& filter_params,
                    const cv::Mat& original,
                    cv::Mat& filtered);

void binary_threshold(const FilterParams& filter_params,
                      const cv::Mat& original,
                      cv::Mat& filtered);

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
    {"overlap", overlap_filter}, // This was done by the one and only Thomas
    {"median_binary", median_binary_filter},
    {"binary", binary_threshold},
};

#endif  // IMAGE_PROCESSING_HPP



