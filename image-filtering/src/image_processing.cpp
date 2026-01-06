#include <image_filters/image_processing.hpp>
#include <image_filters/utilities.hpp>
#include <iostream>




void Flip::apply_filter(const cv::Mat& original, cv::Mat& filtered) const {
    int flip_code = this->filter_params.flip_code;  // 0: x-axis, 1: y-axis, -1: both
    cv::flip(original, filtered, flip_code);
}


void Sharpening::apply_filter(const cv::Mat& original, cv::Mat& filtered) const {
    // Sharpen image
    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    cv::filter2D(original, filtered, -1, kernel);
}

void Unsharpening::apply_filter(const cv::Mat& original, cv::Mat& filtered) const {
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

void Erosion::apply_filter(const cv::Mat& original, cv::Mat& filtered) const {
    apply_erosion(original, filtered, this->filter_params.kernel_size, cv::MORPH_RECT);
}

void Dilation::apply_filter(const cv::Mat& original, cv::Mat& filtered) const{
    apply_dilation(original, filtered, this->filter_params.kernel_size, cv::MORPH_RECT);
}

void WhiteBalance::apply_filter(const cv::Mat& original, cv::Mat& filtered) const{
    double contrast_percentage = this->filter_params.contrast_percentage;
    cv::Ptr<cv::xphoto::SimpleWB> balance = cv::xphoto::createSimpleWB();
    balance->setP(contrast_percentage);
    balance->balanceWhite(original, filtered);
}

void Ebus::apply_filter(const cv::Mat& original, cv::Mat& filtered) const{
    int blur_size = this->filter_params.blur_size;
    int mask_weight = this->filter_params.mask_weight;
    int erosion_size = this->filter_params.erosion_size;
    // Erode image to make blacks more black
    cv::Mat eroded;

    apply_erosion(original, eroded, erosion_size);

    // Make an unsharp mask from original image
    cv::Mat blurred;
    GaussianBlur(original, blurred,
                 cv::Size(2 * blur_size + 1, 2 * blur_size + 1), 0);

    // Compute the unsharp mask
    cv::Mat mask = original - blurred;
    cv::Mat unsharp;

    // Add mask to the eroded image instead of the original
    // Higher weight of mask will create a sharper but more noisy image
    addWeighted(eroded, 1, mask, mask_weight, 0, filtered);
}



void OtsuSegmentation::apply_filter(const cv::Mat& original, cv::Mat& filtered) const{
    bool gamma_auto_correction = this->filter_params.gamma_auto_correction;
    double gamma_auto_correction_weight = this->filter_params.gamma_auto_correction_weight;

    bool otsu_segmentation = this->filter_params.otsu_segmentation;

    if (original.type() == CV_8UC3) { // if the image type is bgr8
        to_weighted_gray(original, filtered, 
                                  this->filter_params.gsc_weight_b,
                                  this->filter_params.gsc_weight_g,
                                  this->filter_params.gsc_weight_r);
    }
    else if (original.type() == CV_8UC1){ original.copyTo(filtered); } // if its mono8
    else {std::cout << "your image type is not matching this filter" << std::endl;}

    if (gamma_auto_correction) { 
        apply_auto_gamma(filtered, gamma_auto_correction_weight);
    } 
 
    if (otsu_segmentation) { 
        apply_otsu(filtered, filtered, false, 255);

        // Apply erosion followed by dilation (opening)

        apply_erosion(filtered, filtered, this->filter_params.erosion_size, cv::MORPH_CROSS);
        apply_dilation(filtered, filtered, this->filter_params.dilation_size, cv::MORPH_CROSS);
    }
}



void Overlap::apply_filter(const cv::Mat& original, cv::Mat& filtered) const
{
    // mono8 only
    CV_Assert(!original.empty());
    CV_Assert(original.type() == CV_8UC1);

    double thr_percent = filter_params.percentage_threshold;

    // First frame (or size/type change): passthrough + cache
    if (!has_prev || prev.empty() ||
        prev.size() != original.size() || prev.type() != original.type())
    {
        original.copyTo(filtered);
        prev = original.clone();
        has_prev = true;
        return;
    }

    // Absolute difference (still mono8)
    cv::Mat diff8u;
    cv::absdiff(original, prev, diff8u);

    // Convert to percent of 8-bit range (0..100)
    cv::Mat percent32f;
    diff8u.convertTo(percent32f, CV_32F, 100.0 / 255.0);

    // Build mask where change > threshold
    cv::Mat mask8u;
    cv::threshold(percent32f, mask8u, thr_percent, 255.0, cv::THRESH_BINARY);
    mask8u.convertTo(mask8u, CV_8U);

    // Output: same as original except zero-out "changed" pixels
    filtered = original.clone();
    filtered.setTo(0, mask8u);

    // Update history (write to cached previous)
    prev = original.clone();
}


void MedianBinary::apply_filter(const cv::Mat& original, cv::Mat& filtered) const{

    apply_median(original, filtered, this->filter_params.kernel_size);
    apply_fixed_threshold(filtered, filtered, this->filter_params.threshold, this->filter_params.invert);
}



void BinaryThreshold::apply_filter(const cv::Mat& original, cv::Mat& filtered) const
{

    CV_Assert(!original.empty());

    const double thresh = this->filter_params.threshold;
    const double maxval = this->filter_params.maxval;
    const bool   invert = this->filter_params.invert;

    // 1) Ensure single-channel
    cv::Mat gray;
    if (original.channels() == 1) {
        gray = original;
    } else {
        cv::cvtColor(original, gray, cv::COLOR_BGR2GRAY);
    }

    // Standardize to 8-bit (safe for thresholding)
    cv::Mat src8;
    if (gray.depth() != CV_8U) {
        // Adjust scaling here if grayscale is not already 0–255
        gray.convertTo(src8, CV_8U);
    } else {
        src8 = gray;
    }  
    // Apply fixed threshold
    const int type = invert ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
    cv::threshold(src8, filtered, thresh, maxval, type);
}




// TODO: Implement your filter here
void Example::apply_filter(const cv::Mat& original, cv::Mat& filtered) const{
    std::string example_str = this->filter_params.example_string;
    int example_int = this->filter_params.example_int;
    DoExample(original,filtered, example_str, example_int);
}
