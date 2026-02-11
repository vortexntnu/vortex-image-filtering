
#include <lib/utilities.hpp>

namespace {  // Making these functions private for this file

// Apply a given gamma to an 8-bit image using a LUT
void applyGammaLUT(cv::Mat& image, double gamma) {
    // Create a lookup table for gamma correction
    cv::Mat lookup(1, 256, CV_8U);
    uchar* p = lookup.ptr();
    for (int i = 0; i < 256; ++i) {
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }

    // Apply the gamma correction using the lookup table
    cv::LUT(image, lookup, image);
}

// Compute a gamma value that pushes the image mean toward mid-gray
double computeAutoGammaFromMean(const cv::Mat& image) {
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
}  // namespace

// Auto-choose a gamma so dark images get lifted and bright images get toned
// down (expects mono8)
// - It sets the mean intensity to 255/2 ≃ 128
// - The correction weight makes all the values weaker(<1) or stronger(>1)
void vortex::image_filtering::apply_auto_gamma(cv::Mat& image,
                                               double correction_weight) {
    double gamma = computeAutoGammaFromMean(image) * correction_weight;
    applyGammaLUT(image, gamma);
}

// Convert BGR image to single-channel grayscale using custom B,G,R weights
// weights = (b, g, r), e.g. (0.114f, 0.587f, 0.299f)
void vortex::image_filtering::to_weighted_gray(const cv::Mat& bgr,
                                               cv::Mat& gray,
                                               double wB,
                                               double wG,
                                               double wR) {
    cv::Matx13f customWeights(wB, wG, wR);
    cv::transform(bgr, gray, customWeights);
}

// Returns the Otsu threshold value chosen by OpenCV (0..255) and outputs the
// thresholded binary image
int vortex::image_filtering::apply_otsu(const cv::Mat& gray8u,
                                        cv::Mat& out,
                                        bool invert,
                                        double maxval) {
    CV_Assert(gray8u.type() == CV_8UC1 &&
              "applyOtsu expects 8-bit single-channel input");

    int ttype = invert ? (cv::THRESH_BINARY_INV | cv::THRESH_OTSU)
                       : (cv::THRESH_BINARY | cv::THRESH_OTSU);

    double thresh =
        cv::threshold(gray8u, out, /*thresh ignored*/ 0.0, maxval, ttype);
    return static_cast<int>(std::round(thresh));
}

void vortex::image_filtering::apply_erosion(const cv::Mat& src,
                                            cv::Mat& filtered,
                                            int size,
                                            int shape) {
    cv::Mat kernel = cv::getStructuringElement(
        shape, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
    cv::erode(src, filtered, kernel);
}

// Basic dilation
void vortex::image_filtering::apply_dilation(const cv::Mat& src,
                                             cv::Mat& dst,
                                             int size,
                                             int shape) {
    cv::Mat kernel = cv::getStructuringElement(
        shape, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
    cv::dilate(src, dst, kernel);
}

// Median filter that preserves original depth if it's unsupported by
// cv::medianBlur. Supported depths: CV_8U, CV_16U, CV_32F For others (e.g.,
// CV_16S, CV_32S, CV_64F) we convert to CV_32F, filter, then convert back.
void vortex::image_filtering::apply_median(const cv::Mat& original,
                                           cv::Mat& filtered,
                                           int kernel_size) {
    CV_Assert(!original.empty());

    // If caller passed 1, just copy
    if (kernel_size == 1) {
        original.copyTo(filtered);
        return;
    }

    // Sanitize kernel size: must be odd and >= 3
    if (kernel_size < 3)
        kernel_size = 3;
    if ((kernel_size & 1) == 0)
        ++kernel_size;

    const int depth = original.depth();
    const bool supported =
        (depth == CV_8U || depth == CV_16U || depth == CV_32F);

    const cv::Mat* src = &original;
    cv::Mat work, out;

    if (!supported) {
        // Convert unsupported depths to CV_32F to avoid clipping (better than
        // going to 8U).
        original.convertTo(work, CV_32F);
        src = &work;
    }

    // Do the median blur (OpenCV supports multi-channel for these depths)
    cv::medianBlur(*src, out, kernel_size);

    // Convert back to original depth if we promoted
    if (!supported) {
        out.convertTo(filtered, depth);
    } else {
        filtered = std::move(out);
    }
}

// Apply a fixed binary threshold.
// - Accepts grayscale or color input (auto-converts to gray).
// - Ensures 8-bit depth for thresholding.
// - Returns a 0/255 mask (CV_8U).
// - Set `invert=true` to get white background & black foreground.
void vortex::image_filtering::apply_fixed_threshold(const cv::Mat& img,
                                                    cv::Mat& filtered,
                                                    int thresh,
                                                    bool invert) {
    if (img.empty()) {
        throw std::invalid_argument(
            "applyFixedThreshold: input image is empty");
    }
    if (thresh < 0 || thresh > 255) {
        throw std::out_of_range(
            "applyFixedThreshold: thresh must be in [0, 255]");
    }

    // Convert to grayscale
    cv::Mat gray;
    if (img.channels() == 3 || img.channels() == 4) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img;
    }

    // Ensure 8-bit
    if (gray.depth() != CV_8U) {
        cv::Mat tmp;
        cv::normalize(gray, tmp, 0, 255, cv::NORM_MINMAX);
        tmp.convertTo(gray, CV_8U);
    }

    // Threshold
    int type = invert ? (cv::THRESH_BINARY_INV) : (cv::THRESH_BINARY);
    cv::threshold(gray, filtered, thresh, 255, type);
}

/*

































                               _.-.
                              /  66\
                             (      `\          Hi, how you doing :)
                             |\\ ,  ,|
                     __      | \\____/
               ,.--"`-.".   /   `---'
           _.-'          '-/      |
       _.-"   |   '-.             |_/_
 ,__.-'  _,.--\      \      ((    /-\
 ',_..--'      `\     \      \\_ /
                 `-,   )      |\'
                   |   |-.,,-" (
                   |   |   `\   `',_
                   )    \    \,(\(\-'
                   \     `-,_
                    \_(\-(\`-`
                       "  "
*/
