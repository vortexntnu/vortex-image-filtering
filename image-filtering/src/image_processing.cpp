#include <image_filters/image_processing.hpp>
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

void applyGammaCorrection(cv::Mat& image, double gamma) {
    // Create a lookup table for gamma correction
    cv::Mat lookup(1, 256, CV_8U);
    uchar* p = lookup.ptr();
    for (int i = 0; i < 256; ++i) {
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }

    // Apply the gamma correction using the lookup table
    cv::LUT(image, lookup, image);
}

double calculateAutoGamma(const cv::Mat& image) {
    // Convert the image to grayscale if it's not already
    cv::Mat grayImage;
    if (image.channels() == 3) {
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    } else {
        grayImage = image;
    }

    // Calculate the mean intensity
    cv::Scalar meanIntensity = mean(grayImage);

    // The ideal mean intensity is 128 (midpoint for 8-bit grayscale images)
    double idealMean = 128.0;
    double currentMean = meanIntensity[0];

    // Automatically set gamma value based on the mean intensity
    double gamma;
    if (currentMean > 0) {
        gamma = log(idealMean / 255.0) / log(currentMean / 255.0);
    } else {
        gamma = 1.0;  // Default gamma if the image has no intensity
    }

    // Ensure gamma is within a reasonable range (e.g., between 0.1 and 3.0)
    gamma = std::max(0.1, std::min(gamma, 3.0));

    return gamma;
}

void otsu_segmentation_filter(const FilterParams& params,
                              const cv::Mat& original,
                              cv::Mat& filtered) {
    bool gamma_auto_correction = params.otsu.gamma_auto_correction;
    double gamma_auto_correction_weight =
        params.otsu.gamma_auto_correction_weight;

    bool otsu_segmentation = params.otsu.otsu_segmentation;

    cv::Mat grayImage;

    cv::Matx13f customWeights(params.otsu.gsc_weight_b,
                              params.otsu.gsc_weight_g,
                              params.otsu.gsc_weight_r);
    cv::transform(original, filtered, customWeights);

    if (gamma_auto_correction) {
        double autoGamma =
            calculateAutoGamma(filtered) * gamma_auto_correction_weight;

        applyGammaCorrection(filtered, autoGamma);
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

        // Initialize variables for Otsu's method
        std::vector<float> sigma2_list(256, 0.0);
        std::vector<float> p(hist.begin<float>(),
                             hist.end<float>());  // Probabilities

        for (int th = 1; th < 256; ++th) {
            // Calculate omega (weights) for foreground and background
            float omega_fg = std::accumulate(p.begin(), p.begin() + th, 0.0f);
            float omega_bg = std::accumulate(p.begin() + th, p.end(), 0.0f);

            // Calculate mu (means) for foreground and background
            float mu_fg = 0, mu_bg = 0;
            for (int i = 0; i < th; ++i) {
                mu_fg += i * p[i];
            }
            for (int i = th; i < 256; ++i) {
                mu_bg += i * p[i];
            }

            if (omega_fg > 0)
                mu_fg /= omega_fg;
            if (omega_bg > 0)
                mu_bg /= omega_bg;

            // Calculate sigma squared and store in list
            sigma2_list[th] = omega_fg * omega_bg * pow(mu_fg - mu_bg, 2);
        }

        // Find the threshold corresponding to the maximum sigma squared
        int optimalThreshold =
            std::max_element(sigma2_list.begin(), sigma2_list.end()) -
            sigma2_list.begin();

        // Apply the threshold to the image
        // cv::Mat binaryImage;
        cv::threshold(filtered, filtered, optimalThreshold, 255,
                      cv::THRESH_BINARY);

        // Apply erosion followed by dilation (opening)
        cv::Mat openedImage;
        int erosionSize = params.otsu.erosion_size;
        cv::Mat erosionKernel = getStructuringElement(
            cv::MORPH_CROSS, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1),
            cv::Point(erosionSize, erosionSize));
        cv::erode(filtered, filtered, erosionKernel);

        int dilation_size = params.otsu.dilation_size;
        cv::Mat dilationKernel = getStructuringElement(
            cv::MORPH_CROSS,
            cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
            cv::Point(dilation_size, dilation_size));
        cv::dilate(filtered, filtered, dilationKernel);
    }
}

void your_filter_function(const FilterParams& filter_params,
                          const cv::Mat &original, 
                          cv::Mat &filtered){
    // Access your filter-specific parameters like this:
    int example_param = filter_params.your_filter.example_param;

    // Implement your filtering logic here

    int flip_code = params.flip.flip_code;  // !!  for testing only
    cv::flip(original, filtered, flip_code);

                        
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
