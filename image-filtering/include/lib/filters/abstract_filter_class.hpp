#ifndef LIB__FILTERS__ABSTRACT_FILTER_CLASS_HPP_
#define LIB__FILTERS__ABSTRACT_FILTER_CLASS_HPP_


#include <opencv2/core.hpp>



namespace vortex::image_filtering {
    class Filter {
    public:
        virtual ~Filter() = default;
        virtual void apply_filter(const cv::Mat& original,
                                cv::Mat& filtered) const = 0;

    protected:
        Filter() = default;
    };
}

#endif  // LIB__FILTERS__ABSTRACT_FILTER_CLASS_HPP_
