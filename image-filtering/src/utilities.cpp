#include <image_filters/utilities.hpp>
#include <iostream>




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


// Auto-choose a gamma so dark images get lifted and bright images get toned down (expects mono8)
// - It sets the mean intensity to 255/2 ≃ 128
// - The weight makes makes all the values weeker(<1) or stronger(>1)
void apply_auto_gamma(cv::Mat& image, double correction_weight) {
    double gamma = computeAutoGammaFromMean(image) * correction_weight;
    applyGammaLUT(image, gamma);
}




// Convert BGR image to single-channel grayscale using custom B,G,R weights
// weights = (b, g, r), e.g. (0.114f, 0.587f, 0.299f)
void to_weighted_gray(const cv::Mat& bgr, cv::Mat& gray, double wB, double wG, double wR) {
    cv::Matx13f customWeights(wB, wG, wR);
    cv::transform(bgr, gray, customWeights);
}




// Returns the Otsu threshold value chosen by OpenCV (0..255) and outputs the thresholded binary image
int apply_otsu(const cv::Mat& gray8u, cv::Mat& out, bool invert, double maxval)
{
    CV_Assert(gray8u.type() == CV_8UC1 && "applyOtsu expects 8-bit single-channel input");

    int ttype = invert ? (cv::THRESH_BINARY_INV | cv::THRESH_OTSU)
                       : (cv::THRESH_BINARY     | cv::THRESH_OTSU);

    double thresh = cv::threshold(gray8u, out, /*thresh ignored*/0.0, maxval, ttype);
    return static_cast<int>(std::round(thresh));
}




// Basic erosion
void apply_erosion(const cv::Mat& src,
                   cv::Mat& filtered,
                   int size,
                   int shape) {
    cv::Mat kernel = cv::getStructuringElement(
        shape,
        cv::Size(2 * size + 1, 2 * size + 1),
        cv::Point(size, size));
    cv::erode(src, filtered, kernel);
}


// Basic dilation
void apply_dilation(const cv::Mat& src,
                    cv::Mat& dst,
                    int size,
                    int shape) {
    cv::Mat kernel = cv::getStructuringElement(
        shape,
        cv::Size(2 * size + 1, 2 * size + 1),
        cv::Point(size, size));
    cv::dilate(src, dst, kernel);
}



// Median filter that preserves original depth if it's unsupported by cv::medianBlur.
// Supported depths: CV_8U, CV_16U, CV_32F
// For others (e.g., CV_16S, CV_32S, CV_64F) we convert to CV_32F, filter, then convert back.
void apply_median(const cv::Mat& original, cv::Mat& filtered, int kernel_size) {
    CV_Assert(!original.empty());


    // If caller passed 1, just copy
    if (kernel_size == 1) {
        original.copyTo(filtered);
        return;
    }

    // Sanitize kernel size: must be odd and >= 3
    if (kernel_size < 3) kernel_size = 3;
    if ((kernel_size & 1) == 0) ++kernel_size;

    const int depth = original.depth();
    const bool supported = (depth == CV_8U || depth == CV_16U || depth == CV_32F);

    const cv::Mat* src = &original;
    cv::Mat work, out;

    if (!supported) {
        // Convert unsupported depths to CV_32F to avoid clipping (better than going to 8U).
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
void apply_fixed_threshold(const cv::Mat& img, cv::Mat& filtered, int thresh, bool invert)
{
    if (img.empty()) {
        throw std::invalid_argument("applyFixedThreshold: input image is empty");
    }
    if (thresh < 0 || thresh > 255) {
        throw std::out_of_range("applyFixedThreshold: thresh must be in [0, 255]");
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









// Takes a strict binary obstacle mask (0/255) and returns a float image 
// where each pixel is the distance (in pixels) to the nearest obstacle.
// Requirements:
// - binObstacles: single-channel CV_8U with values in {0, 255} only.
// - obstaclesAreWhite:
//     true  -> obstacles = 255, free = 0
//     false -> obstacles = 0,   free = 255
// Output:
// - dist: CV_32F distances to nearest obstacle (0 at obstacle pixels).
// Params:
// - type: CV_DIST_L1, CV_DIST_L2, CV_DIST_C, ...
// - maskSize: 3, 5, or CV_DIST_MASK_PRECISE (0)
// Best regards, ChatGPT
void distance_field(const cv::Mat& binObstacles,
                           cv::Mat& dist,
                           bool obstaclesAreWhite,
                           int type,
                           int maskSize)
{
    if (binObstacles.empty())
        throw std::invalid_argument("distance_field_binary: input is empty");

    if (binObstacles.channels() != 1)
        throw std::invalid_argument("distance_field_binary: input must be single-channel");

    if (binObstacles.depth() != CV_8U)
        throw std::invalid_argument("distance_field_binary: input must be CV_8U (0/255)");

    // Validate strict binary: values must be only 0 or 255
    {
        cv::Mat notZero = (binObstacles != 0);
        cv::Mat not255 = (binObstacles != 255);
        cv::Mat invalid = notZero & not255; // pixels that are neither 0 nor 255
        if (cv::countNonZero(invalid) > 0)
            throw std::invalid_argument("distance_field_binary: input must contain only 0 or 255 values");
    }

    // OpenCV distanceTransform computes distance TO the nearest ZERO pixel
    // for NON-ZERO regions. We want distance FROM obstacles.
    // Build freeMask (255 = free, 0 = obstacles).
    cv::Mat freeMask;
    if (obstaclesAreWhite) {
        // obstacles=255 -> free=0, invert to get free=255
        freeMask = 255 - binObstacles;
    } else {
        // obstacles=0, free=255 already correct
        freeMask = binObstacles;
    }

    if (!(maskSize == 3 || maskSize == 5 || maskSize == cv::DIST_MASK_PRECISE))
        maskSize = 3;

    cv::distanceTransform(freeMask, dist, type, maskSize); // dist is CV_32F

    const float cap = 100.f; // pixel
    cv::Mat clipped; cv::min(dist, cap, clipped);
    clipped.convertTo(dist, CV_8U, 255.0f/cap);
    // publish vis8u as "mono8"

}




// TODO: If you need a helper function define it here like this

void apply_example(const cv::Mat& original,
               cv::Mat& filtered,
               std::string example_string,
               int example_int)
{

    filtered = original.clone();

    // Two centered lines: number above string
    const std::string line1 = std::to_string(example_int);
    const std::string line2 = example_string;

    // Text style
    const int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    const double fontScale = 1.0;
    const int thickness = 2;
    const int lineType = cv::LINE_AA;   // smoother; use cv::LINE_8 for hard edges
    const int lineGapPx = 10;           // vertical gap between lines in pixels

    // Measure both lines
    int base1 = 0, base2 = 0;
    const cv::Size sz1 = cv::getTextSize(line1, fontFace, fontScale, thickness, &base1);
    const cv::Size sz2 = cv::getTextSize(line2, fontFace, fontScale, thickness, &base2);

    // Total block size (approx). Heights don't include baseline, so we add baselines for safety.
    const int blockW = std::max(sz1.width, sz2.width);
    const int blockH = (sz1.height + base1) + lineGapPx + (sz2.height + base2);

    // Top-left of the text block so the whole block is centered
    const int blockX = (filtered.cols - blockW) / 2;
    const int blockY = (filtered.rows - blockH) / 2;

    // Baseline positions for each line (y is baseline in putText)
    const int x1 = blockX + (blockW - sz1.width) / 2;
    const int y1 = blockY + sz1.height; // baseline ~ top + height

    const int x2 = blockX + (blockW - sz2.width) / 2;
    const int y2 = y1 + base1 + lineGapPx + sz2.height;

    // Clamp to keep text inside image if needed
    auto clamp = [](int v, int lo, int hi) { return std::max(lo, std::min(v, hi)); };

    const int x1c = clamp(x1, 0, std::max(0, filtered.cols - sz1.width));
    const int y1c = clamp(y1, sz1.height, std::max(sz1.height, filtered.rows - base1));

    const int x2c = clamp(x2, 0, std::max(0, filtered.cols - sz2.width));
    const int y2c = clamp(y2, sz2.height, std::max(sz2.height, filtered.rows - base2));

    // Draw white text on mono8
    cv::putText(filtered, line1, cv::Point(x1c, y1c),
                fontFace, fontScale, cv::Scalar(255), thickness, lineType);

    cv::putText(filtered, line2, cv::Point(x2c, y2c),
                fontFace, fontScale, cv::Scalar(255), thickness, lineType);
}
