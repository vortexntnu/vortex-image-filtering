#include "lib/filters/example.hpp"

void apply_example(const cv::Mat& original,
                   cv::Mat& filtered,
                   std::string example_string,
                   int example_int);

// (New filter): Implement your filter here
void Example::apply_filter(const cv::Mat& original, cv::Mat& filtered) const {
    std::string example_str = this->filter_params.example_string;
    int example_int = this->filter_params.example_int;
    apply_example(original, filtered, example_str, example_int);
}

// (New filter): If you need a helper function define it here like this

void apply_example(const cv::Mat& original,
                   cv::Mat& filtered,
                   std::string example_string,
                   int example_int) {
    filtered = original.clone();

    // Two centered lines: number above string
    const std::string line1 = std::to_string(example_int);
    const std::string line2 = example_string;

    // Text style
    const int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    const double fontScale = 1.0;
    const int thickness = 2;
    const int lineType =
        cv::LINE_AA;           // smoother; use cv::LINE_8 for hard edges
    const int lineGapPx = 10;  // vertical gap between lines in pixels

    // Measure both lines
    int base1 = 0, base2 = 0;
    const cv::Size sz1 =
        cv::getTextSize(line1, fontFace, fontScale, thickness, &base1);
    const cv::Size sz2 =
        cv::getTextSize(line2, fontFace, fontScale, thickness, &base2);

    // Total block size (approx). Heights don't include baseline, so we add
    // baselines for safety.
    const int blockW = std::max(sz1.width, sz2.width);
    const int blockH = (sz1.height + base1) + lineGapPx + (sz2.height + base2);

    // Top-left of the text block so the whole block is centered
    const int blockX = (filtered.cols - blockW) / 2;
    const int blockY = (filtered.rows - blockH) / 2;

    // Baseline positions for each line (y is baseline in putText)
    const int x1 = blockX + (blockW - sz1.width) / 2;
    const int y1 = blockY + sz1.height;  // baseline ~ top + height

    const int x2 = blockX + (blockW - sz2.width) / 2;
    const int y2 = y1 + base1 + lineGapPx + sz2.height;

    // Clamp to keep text inside image if needed
    auto clamp = [](int v, int lo, int hi) {
        return std::max(lo, std::min(v, hi));
    };

    const int x1c = clamp(x1, 0, std::max(0, filtered.cols - sz1.width));
    const int y1c =
        clamp(y1, sz1.height, std::max(sz1.height, filtered.rows - base1));

    const int x2c = clamp(x2, 0, std::max(0, filtered.cols - sz2.width));
    const int y2c =
        clamp(y2, sz2.height, std::max(sz2.height, filtered.rows - base2));

    // Draw white text on mono8
    cv::putText(filtered, line1, cv::Point(x1c, y1c), fontFace, fontScale,
                cv::Scalar(255), thickness, lineType);

    cv::putText(filtered, line2, cv::Point(x2c, y2c), fontFace, fontScale,
                cv::Scalar(255), thickness, lineType);
}
