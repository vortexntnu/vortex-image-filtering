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

// erosion og dilation



#endif