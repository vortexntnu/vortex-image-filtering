#ifndef LIB__filters__EXAMPLE_HPP_
#define LIB__filters__EXAMPLE_HPP_

#include "abstract_filter_class.hpp"

// TODO(New filter): add this structure for your filter

/////////////////////////////
// Example
/////////////////////////////

struct ExampleParams {  // Add filter variables here
    int example_int;
    std::string example_string;
};

class Example : public Filter {
   public:
    explicit Example(ExampleParams params) : filter_params(params) {}
    void apply_filter(const cv::Mat& original, cv::Mat& filtered)
        const override;  // This is the filter itself
   private:
    ExampleParams filter_params;
};

#endif  // LIB__filters__EXAMPLE_HPP_
