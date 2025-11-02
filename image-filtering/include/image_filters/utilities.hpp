#ifndef UTILITIES_HPP
#define UTILITIES_HPP


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>



double calculateAutoGamma(const cv::Mat& image);
void applyGammaCorrection(cv::Mat& image, double gamma);



#endif