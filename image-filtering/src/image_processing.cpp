#include <image_filters/image_processing.hpp>
#include <image_filters/utilities.hpp>
#include <iostream>


void NoFilter::no_filter([[maybe_unused]] const FilterParams& params,
            const cv::Mat& original,
            cv::Mat& filtered) {
    original.copyTo(filtered);
}

void Flip::flip_filter([[maybe_unused]] const FilterParams& params,
                 const cv::Mat& original,
                 cv::Mat& filtered) {
    int flip_code = params.flip.flip_code;  // 0: x-axis, 1: y-axis, -1: both
    cv::flip(original, filtered, flip_code);
}

void Sharpening::sharpening_filter([[maybe_unused]] const FilterParams& params,
                       const cv::Mat& original,
                       cv::Mat& filtered) {
    // Sharpen image
    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    cv::filter2D(original, filtered, -1, kernel);
}

void Unsharpening::unsharpening_filter(const FilterParams& params,
                         const cv::Mat& original,
                         cv::Mat& filtered) {
    int blur_size = params.unsharpening.blur_size;
    // Create a blurred version of the image
    cv::Mat blurred;
    GaussianBlur(original, blurred,
                 cv::Size(2 * blur_size + 1, 2 * blur_size + 1), 0);

    // Compute the unsharp mask
    cv::Mat mask = original - blurred;
    cv::Mat unsharp;

    addWeighted(original, 1, mask, 3, 0, filtered);
}

void Erosion::erosion_filter(const FilterParams& params,
                    const cv::Mat& original,
                    cv::Mat& filtered) {
    apply_erosion(original, filtered, params.eroding.size, cv::MORPH_RECT);
}

void Dilation::dilation_filter(const FilterParams& params,
                     const cv::Mat& original,
                     cv::Mat& filtered) {
    apply_dilation(original, filtered, params.dilating.size, cv::MORPH_RECT);
}

void WhiteBalance::white_balance_filter(const FilterParams& params,
                          const cv::Mat& original,
                          cv::Mat& filtered) {
    double contrast_percentage = params.white_balancing.contrast_percentage;
    cv::Ptr<cv::xphoto::SimpleWB> balance = cv::xphoto::createSimpleWB();
    balance->setP(contrast_percentage);
    balance->balanceWhite(original, filtered);
}

void Ebus::ebus_filter(const FilterParams& params,
                 const cv::Mat& original,
                 cv::Mat& filtered) {
    int blur_size = params.ebus.blur_size;
    int mask_weight = params.ebus.mask_weight;
    // Erode image to make blacks more black
    cv::Mat eroded;

    int erosion_size = params.eroding.size;
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



void Otsu::otsu_segmentation_filter(const FilterParams& params,
                              const cv::Mat& original,
                              cv::Mat& filtered) {
    bool gamma_auto_correction = params.otsu.gamma_auto_correction;
    double gamma_auto_correction_weight =
        params.otsu.gamma_auto_correction_weight;

    bool otsu_segmentation = params.otsu.otsu_segmentation;


    to_weighted_gray(original, filtered, params.otsu.gsc_weight_b,
                              params.otsu.gsc_weight_g,
                              params.otsu.gsc_weight_r);


    if (gamma_auto_correction) { 
        apply_auto_gamma(filtered, gamma_auto_correction_weight);
    } 
 
    if (otsu_segmentation) { 
        apply_otsu(filtered, filtered, false, 255);

        // Apply erosion followed by dilation (opening)

        apply_erosion(filtered, filtered, params.otsu.erosion_size, cv::MORPH_CROSS);
        apply_dilation(filtered, filtered, params.otsu.dilation_size, cv::MORPH_CROSS);
    }
}

// Thomas was here
void Overlap::overlap_filter(const FilterParams& filter_params,
                    const cv::Mat& original,
                    cv::Mat& filtered)
{
    static cv::Mat prev;     // previous mono frame
    static bool has_prev = false;

    // Basic checks (keep it simple; require mono8)
    if (original.empty())
        throw std::invalid_argument("overlap_filter_mono: input is empty");
    if (original.type() != CV_8UC1)
        throw std::invalid_argument("overlap_filter_mono: input must be CV_8UC1 (mono8)");

    if (!has_prev || prev.empty()
        || prev.size() != original.size()
        || prev.type() != original.type())
    {
        original.copyTo(filtered);   // pass-through on first frame / size change
        prev = original.clone();
        has_prev = true;
        return;
    }

    // |cur - prev|
    cv::Mat diff8u;
    cv::absdiff(original, prev, diff8u);

    // Convert percentage threshold to an 8-bit delta threshold
    double pct = std::clamp<double>(filter_params.overlap.percentage_threshold, 0.0, 100.0);
    int delta_thr = static_cast<int>(std::round(pct * 255.0 / 100.0));

    // Mask: changed pixels
    cv::Mat mask;
    cv::threshold(diff8u, mask, delta_thr, 255, cv::THRESH_BINARY);

    // Zero out changed pixels
    filtered = original.clone();
    filtered.setTo(0, mask);

    // Update history
    prev = original.clone();

}

void MedianBinary::median_binary_filter(const FilterParams& filter_params,
                    const cv::Mat& original,
                    cv::Mat& filtered){

    apply_median(original, filtered, filter_params.median_binary.kernel_size);
    apply_fixed_threshold(filtered, filtered, filter_params.median_binary.threshold, filter_params.median_binary.invert);
    distance_field(filtered, filtered);
}



void BinaryThreshold::binary_threshold(const FilterParams& filter_params,
                      const cv::Mat& original,
                      cv::Mat& filtered)
{

    CV_Assert(!original.empty());

    const double thresh = filter_params.binary.threshold;
    const double maxval = filter_params.binary.maxval;
    const bool   invert = filter_params.binary.invert;

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



void Filter::apply_filter(const std::string& filter,
                  const cv::Mat& original,
                  cv::Mat& filtered) {
    if (filter_functions.contains(filter)) {
        ((filter_functions.at(filter)))(params, original, filtered);
    } else {
        original.copyTo(filtered);  // Default to no filter
    }
}


