#include <image_filters/image_processing.hpp>
#include <image_filters/utilities.hpp>
#include <iostream>

void no_filter([[maybe_unused]] const FilterParams& params,
               const cv::Mat& original,
               cv::Mat& filtered) {
    original.copyTo(filtered);
}

void flip_filter([[maybe_unused]] const FilterParams& params,
                 const cv::Mat& original,
                 cv::Mat& filtered) {
    int flip_code = params.flip.flip_code;  // 0: x-axis, 1: y-axis, -1: both
    cv::flip(original, filtered, flip_code);
}

void sharpening_filter([[maybe_unused]] const FilterParams& params,
                       const cv::Mat& original,
                       cv::Mat& filtered) {
    // Sharpen image
    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    cv::filter2D(original, filtered, -1, kernel);
}

void unsharpening_filter(const FilterParams& params,
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

void erosion_filter(const FilterParams& params,
                    const cv::Mat& original,
                    cv::Mat& filtered) {
    int erosion_size = params.eroding.size;
    // Create a structuring element for dilation and erosion
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));

    // Apply erosion to the image
    cv::erode(original, filtered, element);

    // apply_erosion(original, filtered, erosion_size, cv::MORPH_RECT);
}

void dilation_filter(const FilterParams& params,
                     const cv::Mat& original,
                     cv::Mat& filtered) {
    int dilation_size = params.dilating.size;
    // Create a structuring element for dilation and erosion
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
        cv::Point(dilation_size, dilation_size));

    // Apply dilation to the image
    cv::dilate(original, filtered, element);
}

void white_balance_filter(const FilterParams& params,
                          const cv::Mat& original,
                          cv::Mat& filtered) {
    double contrast_percentage = params.white_balancing.contrast_percentage;
    cv::Ptr<cv::xphoto::SimpleWB> balance = cv::xphoto::createSimpleWB();
    balance->setP(contrast_percentage);
    balance->balanceWhite(original, filtered);
}

void ebus_filter(const FilterParams& params,
                 const cv::Mat& original,
                 cv::Mat& filtered) {
    int blur_size = params.ebus.blur_size;
    int mask_weight = params.ebus.mask_weight;
    // Erode image to make blacks more black
    cv::Mat eroded;

    int erosion_size = params.eroding.size;
    // Create a structuring element for dilation and erosion
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));

    // Apply erosion to the image
    cv::erode(original, eroded, element);

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



void otsu_segmentation_filter(const FilterParams& params,
                              const cv::Mat& original,
                              cv::Mat& filtered) {
    bool gamma_auto_correction = params.otsu.gamma_auto_correction;
    double gamma_auto_correction_weight =
        params.otsu.gamma_auto_correction_weight;

    bool otsu_segmentation = params.otsu.otsu_segmentation;


    toWeightedGray(original, filtered, params.otsu.gsc_weight_b,
                              params.otsu.gsc_weight_g,
                              params.otsu.gsc_weight_r);


    if (gamma_auto_correction) { 
        applyAutoGamma(filtered, gamma_auto_correction_weight);
    } 
 
    if (otsu_segmentation) { 
        // Calculate the histogram
        int histSize = 256;
        float range[] = {0, 256};
        const float* histRange = {range};
        cv::Mat hist;
        calcHist(&filtered, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange,
                 true, false);

        // Normalize histogram to get probabilities
        hist /= filtered.total();

        int optimalThreshold = computeOtsuThreshold(hist);

        // Apply the threshold to the image
        cv::threshold(filtered, filtered, optimalThreshold, 255,
                      cv::THRESH_BINARY);

        

        // Apply erosion followed by dilation (opening)

        int erosionSize = params.otsu.erosion_size;
        apply_erosion(filtered, filtered, erosionSize);

        int dilation_size = params.otsu.dilation_size;
        apply_dilation(filtered, filtered, dilation_size);
    }
}

// Thomas was here
void overlap_filter(const FilterParams& filter_params,
                    const cv::Mat& original,
                    cv::Mat& filtered)
{
    static cv::Mat prevR;      // store previous R channel only
    static bool has_prev = false;

    // Extract current R channel
    cv::Mat curR;
    cv::extractChannel(original, curR, 2); // 0=B,1=G,2=R

    if (!has_prev || prevR.empty() || prevR.size()!=curR.size() || prevR.type()!=curR.type()) {
        original.copyTo(filtered);   // first call (or size/type change): pass through
        prevR = curR.clone();        // cache R channel
        has_prev = true;
        return;
    }

    // |cur - prev| on the R channel
    cv::Mat diff8u;
    cv::absdiff(curR, prevR, diff8u);

    // % of full 8-bit range
    cv::Mat percent32f;
    diff8u.convertTo(percent32f, CV_32F, 100.0 / 255.0);

    // Mask: pixels whose % change > threshold
    cv::Mat mask;
    cv::threshold(percent32f, mask, filter_params.overlap.percentage_threshold, 255.0, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8U);

    // Zero out those pixels in the R channel
    filtered = original.clone();
    std::vector<cv::Mat> ch;
    cv::split(filtered, ch);      // ch[2] is R
    ch[2].setTo(0, mask);
    cv::merge(ch, filtered);

    // Update history (R channel only)
    prevR = curR.clone();
}

void median_filter(const FilterParams& filter_params,
                    const cv::Mat& original,
                    cv::Mat& filtered){

    CV_Assert(!original.empty());

    // Validate & sanitize kernel size (must be odd and >= 3)
    int k = filter_params.median.kernel_size;
    if (k < 3) k = 3;
    if ((k & 1) == 0) ++k; // make odd if even

    // cv::medianBlur is not suported for all depths "sais chat"
    // (works for CV_8U, CV_16U, CV_32F)
    const int depth = original.depth();
    const bool supported = (depth == CV_8U || depth == CV_16U || depth == CV_32F);

    const cv::Mat* src = &original;
    cv::Mat tmp;
    if (!supported) {
        // Simple, safe conversion to 8-bit
        original.convertTo(tmp, CV_8U);
        src = &tmp;
    }

    cv::medianBlur(*src, filtered, k);

    // If converted to 8U and want to preserve original depth, converts back here:
    if (!supported) filtered.convertTo(filtered, depth);
}



void binary_threshold(const FilterParams& filter_params,
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



void apply_filter(const std::string& filter,
                  const FilterParams& params,
                  const cv::Mat& original,
                  cv::Mat& filtered) {
    if (filter_functions.contains(filter)) {
        ((filter_functions.at(filter)))(params, original, filtered);
    } else {
        original.copyTo(filtered);  // Default to no filter
    }
}
