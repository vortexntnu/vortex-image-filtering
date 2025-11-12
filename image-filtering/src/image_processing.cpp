#include <image_filters/image_processing.hpp>
#include <image_filters/utilities.hpp>
#include <iostream>



void Unsharpening::apply_filter(const cv::Mat& original, cv::Mat& filtered) const{
    int blur_size = this->filter_params.blur_size;
    // Create a blurred version of the image
    cv::Mat blurred;
    GaussianBlur(original, blurred,
                 cv::Size(2 * blur_size + 1, 2 * blur_size + 1), 0);

    // Compute the unsharp mask
    cv::Mat mask = original - blurred;
    cv::Mat unsharp;

    addWeighted(original, 1, mask, 3, 0, filtered);
}


// void Filter::apply_filter(const std::string& filter,
//                   const cv::Mat& original,
//                   cv::Mat& filtered) {
//     if (filter_functions.contains(filter)) {
//         ((filter_functions.at(filter)))(params, original, filtered);
//     } else {
//         original.copyTo(filtered);  // Default to no filter
//     }
// }


