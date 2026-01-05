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

To extend the functionality of the `image_filtering_node` by adding new filters, follow these steps to ensure compatibility and integration with the existing codebase. There should be //TODO comments where you add your filter:

### Step 1: Add filter to Enum

You should define your filtertype in the filtertype enum in [image_processing.hpp](image-filtering/include/image_filters/image_processing.hpp)

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

### Step 2: Filter string
To access the filter trough the yamal file we need to access it trough a string. You need to add it as a string to parse_filter_type in [image_processing.hpp](image-filtering/include/image_filters/image_processing.hpp)

```cpp
inline FilterType parse_filter_type(std::string s) {
  s = to_lower(std::move(s));
    if (s == "no_filter")      return FilterType::NoFilter;
    if (s == "flip")           return FilterType::Flip;
    if (s == "unsharpening")   return FilterType::Unsharpening;
    ...
    // Add your filter type here:
    
    return FilterType::Unknown;

}
```


### Step 3: Define Filter Parameters

Each filter should have its own set of parameters encapsulated in a structure. Define this structure within [image_processing.hpp](image-filtering/include/image_filters/image_processing.hpp).

```cpp
struct ExampleParams{ 
    // Add necessary filter parameters here
    int example_int;
    std::string example_string;
};
```

### Step 4: Add filter class

Add a Class for your filter inhereting from the Filter class, with the same exact structure as shown below. This should also be in [image_processing.hpp](image-filtering/include/image_filters/image_processing.hpp)
```cpp
class Example: public Filter{
    public:
        explicit Example(ExampleParams params): filter_params(params) {}
        void apply_filter(const cv::Mat& original, cv::Mat& filtered) const override; // This is the filter itself
    private:
        ExampleParams filter_params;
};
```
Here you can add other filter spesific stuff like storing variables that needs to change between runs and so on.



### Step 5: Create the Filter Function

Implement your filter function in [image_processing.cpp](image-filtering/src/image_processing.cpp). This function should take inn the `cv::Mat` objects for the input and the filtered image, and change the filtered one according to your need.

```cpp
void Example::apply_filter(const cv::Mat& original, cv::Mat& filtered) const{
    std::string example_str = this->filter_params.example_string;
    int example_int = this->filter_params.example_int;
    DoExample(original,filtered, example_str, example_int);
}
```
*If you need a helperfunction go to the [helperfunction](#adding-helperfunctions) section of this page.


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

In the constructor of your ROS 2 node, declare each of the new filter parameters using the `declare_parameter` function in [image_filtering_ros.cpp](image-filtering/src/image_filtering_ros.cpp). This sets the default values and prepares the node to accept these parameters at runtime through command line or the YAML configuration file.

```cpp
void ImageFilteringNode::declare_parameters() {
    // Declare your parameters here
    this->declare_parameter<int>("filter_params.example.example_int");
    this->declare_parameter<std::string>("filter_params.example.example_string");
}
```

Then in the same file you make a new case in `set_filter_params` for your filter, to set the variables you just declared.
```cpp
void ImageFilteringNode::set_filter_params() {
    ...
    switch (filter_type){

        ...
        // Add case here
        case FilterType::Example:
    {
        ExampleParams params;
        params.example_int =
            this->get_parameter("filter_params.example.example_int").as_int();
        params.example_string =
            this->get_parameter("filter_params.example.example_string").as_string();

        filter_ptr = std::make_unique<Example>(params);
        break;
    }

    }
}
```



#### Adding Helperfunctions

If you need helperfunctions for your filter, you can add the declaration to [utilities.hpp](image-filtering/include/image_filters/utilities.hpp), and then add the function defenition to  [utilities.cpp](image-filtering/src/utilities.cpp). There wil be TODO comments where you can add them. These cunctions are allredy included in the image_prosessing files.