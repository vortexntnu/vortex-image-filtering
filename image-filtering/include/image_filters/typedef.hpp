#ifndef IMAGE_FILTERS__TYPEDEF_HPP_
#define IMAGE_FILTERS__TYPEDEF_HPP_

#include <map>
#include <numeric>
// #include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/xphoto.hpp>
#include <spdlog/spdlog.h>
#include <fmt/color.h>
#include <string>
#include <utility>




enum class FilterType {  // TODO(New filter): Add filters here
    NoFilter,
    Flip,
    Unsharpening,
    Erosion,
    Dilation,
    WhiteBalancing,
    Ebus,
    Otsu,
    Overlap,
    MedianBinary,
    Binary,

    Example,
    Unknown
};

static constexpr std::pair<std::string_view, FilterType> kFilterMap[] = {
    {"no_filter", FilterType::NoFilter},
    {"flip", FilterType::Flip},
    {"unsharpening", FilterType::Unsharpening},
    {"erosion", FilterType::Erosion},
    {"dilation", FilterType::Dilation},
    {"white_balancing", FilterType::WhiteBalancing},
    {"ebus", FilterType::Ebus},
    {"otsu", FilterType::Otsu},
    {"overlap", FilterType::Overlap},
    {"median_binary", FilterType::MedianBinary},
    {"binary", FilterType::Binary},

    // TODO(New filter): Also add your filter here
    {"example", FilterType::Example},
    {"unknown", FilterType::Unknown}};






    
inline std::string to_lower(std::string s) {
    for (char& ch : s) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return s;
}

inline FilterType parse_filter_type(std::string s) {
    s = to_lower(std::move(s));

    for (auto [name, type] : kFilterMap) {
        if (s == name)
            return type;
    }
    spdlog::warn(fmt::format(fmt::fg(fmt::rgb(200, 180, 50)), "No string connected to that filter type: '{}'. This might be misspelling or you need to add the filter type to kFilterMap in image_processing.hpp", s));
    return FilterType::Unknown;
}

inline std::string_view filtertype_to_string(FilterType t) {
    for (auto [name, type] : kFilterMap) {
        if (t == type)
            return name;
    }
    spdlog::warn(fmt::format(fmt::fg(fmt::rgb(200, 180, 50)), " No string connected to your filter type. To fix "
                 "this add the string and filter type to kFilterMap"));
    return "unknown";
}





#endif // IMAGE_FILTERS__TYPEDEF_HPP_