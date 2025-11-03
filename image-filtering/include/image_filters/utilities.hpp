#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>




void applyAutoGamma(cv::Mat& image, double correction_weight);
void toWeightedGray(const cv::Mat& bgr, cv::Mat& gray, double wB, double wG, double wR);

int computeOtsuThreshold(const cv::Mat& hist_prob);

int applyOtsu(const cv::Mat& gray8u, cv::Mat& out, bool invert = false, double maxval = 255.0);

void apply_erosion(const cv::Mat& src, cv::Mat& dst, int size, int shape = cv::MORPH_RECT);
void apply_dilation(const cv::Mat& src, cv::Mat& dst, int size, int shape=cv::MORPH_RECT);


#endif