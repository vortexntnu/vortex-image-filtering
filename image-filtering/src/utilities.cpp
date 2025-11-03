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
void applyAutoGamma(cv::Mat& image, double correction_weight) {
    double gamma = computeAutoGammaFromMean(image) * correction_weight;
    applyGammaLUT(image, gamma);
}




// Convert BGR image to single-channel grayscale using custom B,G,R weights
// weights = (b, g, r), e.g. (0.114f, 0.587f, 0.299f)
void toWeightedGray(const cv::Mat& bgr, cv::Mat& gray, double wB, double wG, double wR) {
    cv::Matx13f customWeights(wB, wG, wR);
    cv::transform(bgr, gray, customWeights);
}




// Computes the Otsu optimal threshold from a 256-bin normalized grayscale histogram.
// `hist_prob` must contain probabilities (sum ≈ 1). For every possible threshold t,
// the function splits the histogram into foreground [0, t) and background [t, 255],
// computes their weights and means, evaluates the between-class variance, and returns
// the t that maximizes it. A good threshold is one that makes two sizeable groups that
// are well separated in intensity:
//    - Best regards ChatGPT
int computeOtsuThreshold(const cv::Mat& hist_prob)
{
    // Initialize variables for Otsu's method
    std::vector<float> sigma2_list(256, 0.0);
    std::vector<float> p(hist_prob.begin<float>(), hist_prob.end<float>());  // Probabilities

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

    return int(std::max_element(sigma2_list.begin(), sigma2_list.end()) - sigma2_list.begin());
}



// Returns the Otsu threshold value chosen by OpenCV (0..255) and outputs the thresholded binary image
int applyOtsu(const cv::Mat& gray8u, cv::Mat& out, bool invert, double maxval)
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































//                               _.-.
//                              /  66\
//                             (      `\          Hi, how you doin :)
//                             |\\ ,  ,|
//                     __      | \\____/
//               ,.--"`-.".   /   `---'
//           _.-'          '-/      |
//       _.-"   |   '-.             |_/_
// ,__.-'  _,.--\      \      ((    /-\
// ',_..--'      `\     \      \\_ /
//                 `-,   )      |\' 
//                   |   |-.,,-" (  
//                   |   |   `\   `',_
//                   )    \    \,(\(\-'
//                   \     `-,_
//                    \_(\-(\`-`
//                       "  "

