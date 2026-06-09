#ifndef IMAGE_FILTERING__LIB__FILTERS__CLAHE_LAB_HPP_
#define IMAGE_FILTERING__LIB__FILTERS__CLAHE_LAB_HPP_

#include <opencv2/imgproc.hpp>
#include <vector>
#include "abstract_filter_class.hpp"
/////////////////////////////
// CLAHE on LAB color space
/////////////////////////////
// Contrast Limited Adaptive Histogram Equalization applied only to the L
// (lightness) channel of the CIELAB color space. Working in LAB keeps the
// chrominance (a, b) untouched, so local contrast/visibility is boosted without
// shifting hues - useful for hazy, low-contrast underwater imagery.
namespace vortex::image_filtering {
struct ClaheLabParams {
    // Threshold for contrast limiting. Higher values give more contrast (and
    // more amplified noise).
    double clip_limit;
    // Number of tiles per row/column. The image is divided into a
    // tile_grid_size x tile_grid_size grid and equalized per tile.
    int tile_grid_size;
};

class ClaheLab : public Filter {
   public:
    explicit ClaheLab(ClaheLabParams params) : filter_params_(params) {}
    void apply_filter(const cv::Mat& original,
                      cv::Mat& filtered) const override;

   private:
    ClaheLabParams filter_params_;
};

inline void ClaheLab::apply_filter(const cv::Mat& original,
                                   cv::Mat& filtered) const {
    cv::Ptr<cv::CLAHE> clahe =
        cv::createCLAHE(this->filter_params_.clip_limit,
                        cv::Size(this->filter_params_.tile_grid_size,
                                 this->filter_params_.tile_grid_size));

    // Grayscale input has no color space to convert; equalize directly.
    if (original.channels() == 1) {
        clahe->apply(original, filtered);
        return;
    }

    // The node delivers images in RGB order (input_encoding: rgb8).
    cv::Mat lab;
    cv::cvtColor(original, lab, cv::COLOR_RGB2Lab);

    std::vector<cv::Mat> lab_channels;
    cv::split(lab, lab_channels);

    clahe->apply(lab_channels[0], lab_channels[0]);

    cv::merge(lab_channels, lab);
    cv::cvtColor(lab, filtered, cv::COLOR_Lab2RGB);
}
}  // namespace vortex::image_filtering
#endif  // IMAGE_FILTERING__LIB__FILTERS__CLAHE_LAB_HPP_
