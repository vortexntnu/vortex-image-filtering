#include <image_filters/image_filtering_ros.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <image_filters/image_filtering_ros_utils.hpp>


ImageFilteringNode::ImageFilteringNode(const rclcpp::NodeOptions& options)
    : Node("image_filtering_node", options) {
    declare_parameters();
    check_and_subscribe_to_image_topic();
    set_filter_params();
    initialize_parameter_handler();
    check_and_publish_to_output_topic();

}

// void ImageFilteringNode::declare_parameters() {
//     this->declare_parameter<std::string>("sub_topic");
//     this->declare_parameter<std::string>("pub_topic");
//     this->declare_parameter<std::string>("output_encoding");
//     this->declare_parameter<std::string>("input_encoding");
//     this->declare_parameter<std::string>("filter_params.filter_type");

//     this->declare_parameter<int>("filter_params.flip.flip_code");
//     this->declare_parameter<int>("filter_params.unsharpening.blur_size");
//     this->declare_parameter<int>("filter_params.erosion.size");
//     this->declare_parameter<int>("filter_params.dilation.size");
//     this->declare_parameter<double>(
//         "filter_params.white_balancing.contrast_percentage");
//     this->declare_parameter<int>("filter_params.ebus.erosion_size");
//     this->declare_parameter<int>("filter_params.ebus.blur_size");
//     this->declare_parameter<int>("filter_params.ebus.mask_weight");
//     this->declare_parameter<bool>("filter_params.otsu.gamma_auto_correction");
//     this->declare_parameter<double>(
//         "filter_params.otsu.gamma_auto_correction_weight");
//     this->declare_parameter<bool>("filter_params.otsu.otsu_segmentation");
//     this->declare_parameter<double>("filter_params.otsu.gsc_weight_r");
//     this->declare_parameter<double>("filter_params.otsu.gsc_weight_g");
//     this->declare_parameter<double>("filter_params.otsu.gsc_weight_b");
//     this->declare_parameter<int>("filter_params.otsu.erosion_size");
//     this->declare_parameter<int>("filter_params.otsu.dilation_size");

//     this->declare_parameter<double>(
//         "filter_params.overlap.percentage_threshold");

//     this->declare_parameter<double>("filter_params.binary.threshold");
//     this->declare_parameter<double>("filter_params.binary.maxval");
//     this->declare_parameter<bool>("filter_params.binary.invert");

//     this->declare_parameter<int>("filter_params.median_binary.kernel_size");
//     this->declare_parameter<int>("filter_params.median_binary.threshold");
//     this->declare_parameter<bool>("filter_params.median_binary.invert");

//     // TODO(New filter): Declare parameters set for your filter here
//     this->declare_parameter<int>("filter_params.example.example_int");
//     this->declare_parameter<std::string>(
//         "filter_params.example.example_string");
// }

void ImageFilteringNode::set_filter_params() {
    std::string filter_type_string =
        this->get_parameter("filter_params.filter_type").as_string();
    FilterType filter_type = parse_filter_type(filter_type_string);

    switch (filter_type) {

        case FilterType::NoFilter: {
            filter_ptr = std::make_unique<NoFilter>();
            break;
        }

        case FilterType::Unsharpening: {
            UnsharpeningParams params;
            params.blur_size =
                declare_and_get<int>("filter_params.unsharpening.blur_size");

            filter_ptr = std::make_unique<Unsharpening>(params);
            break;
        }

        case FilterType::Flip: {
            FlipParams params;
            params.flip_code =
                declare_and_get<int>("filter_params.flip.flip_code");

            filter_ptr = std::make_unique<Flip>(params);
            break;
        }

        case FilterType::Erosion: {
            ErosionParams params;
            params.kernel_size =
                declare_and_get<int>("filter_params.erosion.size");

            filter_ptr = std::make_unique<Erosion>(params);
            break;
        }

        case FilterType::Dilation: {
            DilationParams params;
            params.kernel_size =
                declare_and_get<int>("filter_params.dilation.size");

            filter_ptr = std::make_unique<Dilation>(params);
            break;
        }

        case FilterType::WhiteBalancing: {
            WhiteBalanceParams params;
            params.contrast_percentage =
                declare_and_get<double>(
                    "filter_params.white_balancing.contrast_percentage");

            filter_ptr = std::make_unique<WhiteBalance>(params);
            break;
        }

        case FilterType::Ebus: {
            EbusParams params;
            params.erosion_size =
                declare_and_get<int>("filter_params.ebus.erosion_size");
            params.blur_size =
                declare_and_get<int>("filter_params.ebus.blur_size");
            params.mask_weight =
                declare_and_get<int>("filter_params.ebus.mask_weight");

            filter_ptr = std::make_unique<Ebus>(params);
            break;
        }

        case FilterType::Otsu: {
            OtsuSegmentationParams params;
            params.gamma_auto_correction =
                declare_and_get<bool>("filter_params.otsu.gamma_auto_correction");
            params.gamma_auto_correction_weight =
                declare_and_get<double>(
                    "filter_params.otsu.gamma_auto_correction_weight");
            params.otsu_segmentation =
                declare_and_get<bool>("filter_params.otsu.otsu_segmentation");
            params.gsc_weight_r =
                declare_and_get<double>("filter_params.otsu.gsc_weight_r");
            params.gsc_weight_g =
                declare_and_get<double>("filter_params.otsu.gsc_weight_g");
            params.gsc_weight_b =
                declare_and_get<double>("filter_params.otsu.gsc_weight_b");
            params.erosion_size =
                declare_and_get<int>("filter_params.otsu.erosion_size");
            params.dilation_size =
                declare_and_get<int>("filter_params.otsu.dilation_size");

            filter_ptr = std::make_unique<OtsuSegmentation>(params);
            break;
        }

        case FilterType::Overlap: {
            OverlapParams params;
            params.percentage_threshold =
                declare_and_get<double>(
                    "filter_params.overlap.percentage_threshold");

            filter_ptr = std::make_unique<Overlap>(params);
            break;
        }

        case FilterType::MedianBinary: {
            MedianBinaryParams params;
            params.kernel_size =
                declare_and_get<int>("filter_params.median_binary.kernel_size");
            params.threshold =
                declare_and_get<int>("filter_params.median_binary.threshold");
            params.invert =
                declare_and_get<bool>("filter_params.median_binary.invert");

            filter_ptr = std::make_unique<MedianBinary>(params);
            break;
        }

        case FilterType::Binary: {
            BinaryThresholdParams params;
            params.threshold =
                declare_and_get<double>("filter_params.binary.threshold");
            params.maxval =
                declare_and_get<double>("filter_params.binary.maxval");
            params.invert =
                declare_and_get<bool>("filter_params.binary.invert");

            filter_ptr = std::make_unique<BinaryThreshold>(params);
            break;
        }

        // TODO(New filter): Add your filter case here:
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

        default:;
            if (filter_type == FilterType::Unknown){
                spdlog::warn(fmt::format(fmt::fg(fmt::rgb(200, 180, 50)),
                        "Invalid filter type received: {}. Using default: no_filter.",
                        filter_type_string));
            }
            else{
                spdlog::warn(fmt::format(fmt::fg(fmt::rgb(200, 180, 50)),
                    "Filterparams has not been set for your chosen filter "
                    "{}. "
                    "To fix this add your filter to "
                    "ImageFilteringNode::set_filter_params(). "
                    "Using default: no_filter.",
                    filter_type_string));
            }

            filter_ptr = std::make_unique<NoFilter>();
            filter_type = FilterType::NoFilter;

            return;
    }




    spdlog::info(fmt::format(fmt::fg(fmt::rgb(31, 161, 221)),"Using filter: {}",
                filter_type_string));
}

// void ImageFilteringNode::check_and_subscribe_to_image_topic() {
//     std::string image_topic = this->get_parameter("sub_topic").as_string();
//     if (image_topic_ != image_topic) {
//         rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
//         auto qos_sensor_data = rclcpp::QoS(
//             rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

//         image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//             image_topic, qos_sensor_data,
//             std::bind(&ImageFilteringNode::image_callback, this, _1));
//         image_topic_ = image_topic;
//         spdlog::info("Subscribed to image topic: {}", image_topic);
//     }
// }

// void ImageFilteringNode::check_and_publish_to_output_topic(){
//     std::string pub_topic = this->get_parameter("pub_topic").as_string();
//     if (pub_topic_ != pub_topic){
//         rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data; // This is some quality of service stuf (prefers low latency over quality)
//         auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile); // ROS keeps only the newest image


//         image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(pub_topic, qos_sensor_data);

//         pub_topic_ = pub_topic;
//         spdlog::info("Publishing to image topic: {}", pub_topic);
//     } 
// }

void ImageFilteringNode::initialize_parameter_handler() {
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto parameter_event_callback =
        [this](const rcl_interfaces::msg::ParameterEvent& event) -> void {
        this->on_parameter_event(event);
    };

    param_cb_handle_ =
        param_handler_->add_parameter_event_callback(parameter_event_callback);
}

void ImageFilteringNode::on_parameter_event(
    const rcl_interfaces::msg::ParameterEvent& event) {
    auto node_name = this->get_fully_qualified_name();

    if (event.node != node_name) {
        return;
    }
    spdlog::info("Parameter event for node: {}", node_name);
    for (const auto& changed_parameter : event.changed_parameters) {
        if (changed_parameter.name.find("sub_topic") == 0)
            check_and_subscribe_to_image_topic();
        if (changed_parameter.name.find("filter_params") == 0)
            set_filter_params();
        if (changed_parameter.name.find("pub_topic") == 0)
            check_and_publish_to_output_topic();
    }
}

void ImageFilteringNode::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;

    std::string input_encoding =
        this->get_parameter("input_encoding").as_string();

    if (input_encoding.empty()) {
        input_encoding = msg->encoding;  // Default to the input image encoding
    }

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, input_encoding);

        if (cv_ptr->image.empty()) {
            spdlog::error("Received empty image, skipping processing.");
            return;
        }
    } catch (cv_bridge::Exception& e) {
        spdlog::error("cv_bridge exception: {}", e.what());
        ;
    }

    cv::Mat input_image = cv_ptr->image;
    cv::Mat filtered_image;

    try{
        filter_ptr->apply_filter(input_image, filtered_image);
    } 
    catch (const cv::Exception& e) {
        spdlog::error(fmt::format(fmt::fg(fmt::rgb(31, 161, 221)), "OpenCV error while applying filter: {}", e.what()));
        filtered_image = input_image.clone();  // fallback to no filter
    }
    catch (const std::exception& e) {
        spdlog::error(fmt::format(fmt::fg(fmt::rgb(31, 161, 221)),"Error while applying filter: {}", e.what()));
        filtered_image = input_image.clone();
    }

    std::string output_encoding =
        this->get_parameter("output_encoding").as_string();
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(msg->header, output_encoding, filtered_image)
        .toImageMsg(*message);

    image_pub_->publish(std::move(message));
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImageFilteringNode)
