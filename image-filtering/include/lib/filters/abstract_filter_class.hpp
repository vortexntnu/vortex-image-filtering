#ifndef LIB__FILTERS__ABSTRACT_FILTER_CLASS_HPP_
#define LIB__FILTERS__ABSTRACT_FILTER_CLASS_HPP_

// #include <opencv2/core.hpp> 

#include <fmt/color.h>
#include <spdlog/spdlog.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>
#include <string>
#include <utility>

class Filter {
   public:
    virtual ~Filter() = default;
    virtual void apply_filter(const cv::Mat& original,
                              cv::Mat& filtered) const = 0;

   protected:
    Filter() = default;
};

#endif  // LIB__FILTERS__ABSTRACT_FILTER_CLASS_HPP_
