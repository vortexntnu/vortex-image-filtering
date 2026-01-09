#ifndef IMAGE_FILTERS__UTILITIES_HPP_
#define IMAGE_FILTERS__UTILITIES_HPP_

#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>
#include <string>

// Auto-choose a gamma so dark images get lifted and bright images get toned
// down (expects mono8)
void apply_auto_gamma(cv::Mat& image, double correction_weight);

// Converts a BGR image to grayscale using custom channel weights (wB, wG, wR).
void to_weighted_gray(const cv::Mat& bgr,
                      cv::Mat& gray,
                      double wB,
                      double wG,
                      double wR);

// Applies Otsu’s automatic thresholding and returning the threshold
int apply_otsu(const cv::Mat& gray8u,
               cv::Mat& out,
               bool invert = false,
               double maxval = 255.0);

// Performs morphological erosion (shrinks bright regions / removes small white
// noise)
void apply_erosion(const cv::Mat& src,
                   cv::Mat& dst,
                   int size,
                   int shape = cv::MORPH_RECT);

// Performs morphological dilation (grows bright regions / fills small holes)
// opposite of erosion
void apply_dilation(const cv::Mat& src,
                    cv::Mat& dst,
                    int size,
                    int shape = cv::MORPH_RECT);

// Applies a median blur with `kernel_size` to reduce salt-and-pepper noise
// while preserving edges.
void apply_median(const cv::Mat& original, cv::Mat& filtered, int kernel_size);

// Apply a fixed binary threshold.
// - Accepts grayscale or color input (auto-converts to gray).
// - Ensures 8-bit depth for thresholding.
// - Returns a 0/255 mask (CV_8U).
// - Set `invert=true` to get white background & black foreground.
void apply_fixed_threshold(const cv::Mat& img,
                           cv::Mat& filtered,
                           int thresh,
                           bool invert = false);

// This does not work properly (Chat has failed me)
void distance_field(const cv::Mat& binObstacles,
                    cv::Mat& dist,
                    bool obstaclesAreWhite = true,
                    int type = cv::DIST_L2,
                    int maskSize = 3);  // DIST_L2 is normal euclidean

// TODO(New filter): If you need a helper function have the signature here
void apply_example(const cv::Mat& original,
                   cv::Mat& filtered,
                   std::string example_string,
                   int example_int);

#endif  // IMAGE_FILTERS__UTILITIES_HPP_
