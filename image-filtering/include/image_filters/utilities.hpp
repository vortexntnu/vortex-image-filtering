#ifndef UTILITIES_HPP
#define UTILITIES_HPP


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>




void applyAutoGamma(cv::Mat& image, double correction_weight);

// weighted transform, gamma correction, thresholding, erosion og dilation



#endif