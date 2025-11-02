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







//                               _.-.
//                              /  66\
//                             (      `\
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

