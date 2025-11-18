#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>




void apply_auto_gamma(cv::Mat& image, double correction_weight);
void to_weighted_gray(const cv::Mat& bgr, cv::Mat& gray, double wB, double wG, double wR);

int apply_otsu(const cv::Mat& gray8u, cv::Mat& out, bool invert = false, double maxval = 255.0);

void apply_erosion(const cv::Mat& src, cv::Mat& dst, int size, int shape = cv::MORPH_RECT);
void apply_dilation(const cv::Mat& src, cv::Mat& dst, int size, int shape=cv::MORPH_RECT);

void apply_median(const cv::Mat& original, cv::Mat& filtered, int kernel_size);

void apply_fixed_threshold(const cv::Mat& img, cv::Mat& filtered, int thresh, bool invert = false);


// TODO Fix, does not work properly (Chat has failed me)
void distance_field(const cv::Mat& binObstacles, cv::Mat& dist, bool obstaclesAreWhite = true, int type = cv::DIST_L2, int maskSize = 3); // DIST_L2 is normal euclidian

#endif