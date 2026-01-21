# Image Filtering Node
[![Industrial CI](https://github.com/vortexntnu/vortex-image-filtering/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/vortex-image-filtering/actions/workflows/industrial-ci.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/vortexntnu/vortex-image-filtering/main.svg)](https://results.pre-commit.ci/latest/github/vortexntnu/vortex-image-filtering/main)
[![codecov](https://codecov.io/github/vortexntnu/vortex-image-filtering/graph/badge.svg?token=6XHprkpUsR)](https://codecov.io/github/vortexntnu/vortex-image-filtering)

The `image_filtering_node` is a ROS 2 node developed in the `vortex::image_filters` namespace. It is designed to subscribe to image topics, apply various image filters using OpenCV, and publish the filtered images back to ROS.

## Features

- **Multiple Filters**: Supports sharpening, unsharpening, eroding, dilating, white balancing, and a custom "ebus" filter.
- **Dynamic Reconfiguration**: Allows for runtime changes to filter parameters and subscribed image topics via ROS 2 parameters.
- **Parameter-Driven Configuration**: Configures filter types and specific attributes through ROS 2 parameters.

## Configuration

Configure the node using ROS 2 parameters:

- **`image_topic`**: Topic from which the node will subscribe to image messages.
- **`filter_params.filter_type`**: Specifies the filter to apply. Current options include:
  - `nofilter`
  - `sharpening`
  - `unsharpening`
  - `eroding`
  - `dilating`
  - `white_balancing`
  - `ebus`
- **Other filter-specific parameters** such as `blur_size`, `size`, `contrast_percentage`, can be configured through parameter `filter_params`.{filter_name}.{param_name}

Parameters can be set through a YAML file or dynamically adjusted at runtime.

## Additional filters

## Implementing New Filters

To extend the functionality of the `image_filtering_node` by adding new filters, follow these steps to ensure compatibility and integration with the existing codebase. There should be //TODO(New filter) comments where you add your filter:

### Step 1: Filter Enum

You should define your filtertype in the filtertype enum in [typedef.hpp](image-filtering/include/lib/typedef.hpp)

```cpp
enum class FilterType {
  NoFilter,
  Flip,
  Unsharpening,
  Erosion,
  Dilation,
  ...
  // Add your filter here
};
```
To access the filter through the yaml file we need to access it through a string. In the same file you need to add it as an item in the kFilterMap.

```cpp
static constexpr std::pair<std::string_view, FilterType> kFilterMap[] = {
    {"no_filter",       FilterType::NoFilter},
    {"flip",            FilterType::Flip},
    {"unsharpening",    FilterType::Unsharpening},
    ...

    // Add your filter here
    {"example",         FilterType::Example},
    {"unknown",         FilterType::Unknown}
};
```

### Step 2: Make the filter header
Each filter should have its own headerfile asosiated with it. You can add this in the [filters](image-filtering/include/lib/filters), and name it the same as your filter (your_filter.hpp). In this file you start with adding these lines (swaping out example with your filter):

```cpp
#ifndef LIB__filters__EXAMPLE_HPP_
#define LIB__filters__EXAMPLE_HPP_
#include "abstract_filter_class.hpp"


// Insert code here ...


#endif // LIB__filters__EXAMPLE_HPP_
```
This new file needs to be added to [all_filters.hpp](image-filtering/include/lib/filters/all_filters.hpp).
```cpp
#ifndef LIB__FILTERS__EXAMPLE__HPP_
#define LIB__FILTERS__EXAMPLE__HPP_

#include "lib/filters/example.hpp"
#include "lib/filters/no_filter.hpp"
              ...
// Add this 
#include "lib/filters/your_filter.hpp"


#endif // LIB__FILTERS__EXAMPLE__HPP_
```


### Step 3: Define Filter Parameters

Each filter should have its own set of parameters encapsulated in a structure. Define this structure within your_filter.hpp.

```cpp
struct ExampleParams{
    // Add necessary filter parameters here
    int example_int;
    std::string example_string;
};
```

### Step 4: Add filter class

Below the filter parameters add a Class for your filter inheriting from the Filter class, with the same structure as shown below. This should also be in [image_processing.hpp](image-filtering/include/image_filters/image_processing.hpp)
```cpp
class Example: public Filter{
    public:
        explicit Example(ExampleParams params): filter_params(params) {}
        void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override; // This is the filter itself
    private:
        ExampleParams filter_params;
};
```
Here you can add other filter specific stuff like storing variables that need to change between runs and so on.
When this is done it should look like [this](image-filtering/include/lib/filters/example.hpp).


### Step 5: Define the filter function

You can do this in two differente ways. If your filter is big you can add a cpp file for your filter, explaned in the [helperfunctions](#helper-functions) section of this page. Otherwise you can add the function defenition just below the class defenition like this.

```cpp
inline void Example::apply_filter(const cv::Mat& original, cv::Mat& filtered) const{
    std::string example_str = this->filter_params.example_string;
    int example_int = this->filter_params.example_int;
    DoExample(original,filtered, example_str, example_int);
}
```


### Step 6: Add to config file

In the [image_filtering_params.yaml](image-filtering/config/image_filtering_params.yaml) file you add your filter and filterparameters for easily interfacing with the filters:

```yaml
    filter_params:
        filter_type: "example"

        flip:
            flip_code: 1
        ...
        # Add your filter type here

        example:
            example_int: 5
            example_string: "This is an example"
```


### Step 7: Declare and Assign Parameters

Now we need to use the right filter and set the variables for your filter. We do that by making a new case in `set_filter_params`, in [image_filtering_ros.cpp](image-filtering/src/ros/image_filtering_ros.cpp), for your filter. 

```cpp
void ImageFilteringNode::set_filter_params() {
    ...
    switch (filter_type){

        ...

        case FilterType::Example: {
            ExampleParams params;
            params.example_int =
                declare_and_get<int>("filter_params.example.example_int");
            params.example_string =
                declare_and_get<std::string>(
                    "filter_params.example.example_string");

            filter_ptr = std::make_unique<Example>(params);
            break;
        }
    }
}
```



### Helper functions

Is your filter to big, is it a chunky boy, are you in need of space, then you have come to the right place!!! 

#### Define the apply filter function

If the Example::apply_filter function gets to big you can add a cpp file named the same as your hpp file (your_filter.cpp) in the [filters](image-filtering/src/lib/filters) folder. There you add #include "lib/filters/your_filter.hpp" at the top. Then you can define your filter there. Here is an [example](image-filtering/src/lib/filters/example.cpp)

#### Make and use helperfunctions


If you are making a function that is useful for many differente filters, then you can add the declaration to [utilities.hpp](image-filtering/include/lib/utilities.hpp), and the defenition to [utilities.cpp](image-filtering/src/lib/utilities.cpp). Make a description off the function above the declaration. To use this in the filter add `#include "image-filtering/include/lib/utilities.hpp"` to your_filter.hpp

