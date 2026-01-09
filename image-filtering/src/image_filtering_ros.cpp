#include <image_filters/image_filtering_ros.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using std::placeholders::_1;

ImageFilteringNode::ImageFilteringNode(const rclcpp::NodeOptions& options)
    : Node("image_filtering_node", options) {
    declare_parameters();
    check_and_subscribe_to_image_topic();
    set_filter_params();
    initialize_parameter_handler();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    std::string pub_topic = this->get_parameter("pub_topic").as_string();
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        pub_topic, qos_sensor_data);
}

void ImageFilteringNode::declare_parameters() {
    this->declare_parameter<std::string>("sub_topic");
    this->declare_parameter<std::string>("pub_topic");
    this->declare_parameter<std::string>("output_encoding");
    this->declare_parameter<std::string>("input_encoding");
    this->declare_parameter<std::string>("filter_params.filter_type");
    this->declare_parameter<int>("filter_params.flip.flip_code");
    this->declare_parameter<int>("filter_params.unsharpening.blur_size");
    this->declare_parameter<int>("filter_params.erosion.size");
    this->declare_parameter<int>("filter_params.dilation.size");
    this->declare_parameter<double>(
        "filter_params.white_balancing.contrast_percentage");
    this->declare_parameter<int>("filter_params.ebus.erosion_size");
    this->declare_parameter<int>("filter_params.ebus.blur_size");
    this->declare_parameter<int>("filter_params.ebus.mask_weight");
    this->declare_parameter<bool>("filter_params.otsu.gamma_auto_correction");
    this->declare_parameter<double>(
        "filter_params.otsu.gamma_auto_correction_weight");
    this->declare_parameter<bool>("filter_params.otsu.otsu_segmentation");
    this->declare_parameter<double>("filter_params.otsu.gsc_weight_r");
    this->declare_parameter<double>("filter_params.otsu.gsc_weight_g");
    this->declare_parameter<double>("filter_params.otsu.gsc_weight_b");
    this->declare_parameter<int>("filter_params.otsu.erosion_size");
    this->declare_parameter<int>("filter_params.otsu.dilation_size");

    this->declare_parameter<double>(
        "filter_params.overlap.percentage_threshold");

    this->declare_parameter<double>("filter_params.binary.threshold");
    this->declare_parameter<double>("filter_params.binary.maxval");
    this->declare_parameter<bool>("filter_params.binary.invert");

    this->declare_parameter<int>("filter_params.median_binary.kernel_size");
    this->declare_parameter<int>("filter_params.median_binary.threshold");
    this->declare_parameter<bool>("filter_params.median_binary.invert");

    // TODO(New filter): Declare parameters set for your filter here
    this->declare_parameter<int>("filter_params.example.example_int");
    this->declare_parameter<std::string>(
        "filter_params.example.example_string");
}

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
                this->get_parameter("filter_params.unsharpening.blur_size")
                    .as_int();

            filter_ptr = std::make_unique<Unsharpening>(params);
            break;
        }

        case FilterType::Flip: {
            FlipParams params;
            params.flip_code =
                this->get_parameter("filter_params.flip.flip_code").as_int();

            filter_ptr = std::make_unique<Flip>(params);
            break;
        }

        case FilterType::Erosion: {
            ErosionParams params;
            params.kernel_size =
                this->get_parameter("filter_params.erosion.size").as_int();

            filter_ptr = std::make_unique<Erosion>(params);
            break;
        }

        case FilterType::Dilation: {
            DilationParams params;
            params.kernel_size =
                this->get_parameter("filter_params.dilation.size").as_int();

            filter_ptr = std::make_unique<Dilation>(params);
            break;
        }

        case FilterType::WhiteBalancing: {
            WhiteBalanceParams params;
            params.contrast_percentage =
                this->get_parameter(
                        "filter_params.white_balancing.contrast_percentage")
                    .as_double();

            filter_ptr = std::make_unique<WhiteBalance>(params);
            break;
        }

        case FilterType::Ebus: {
            EbusParams params;
            params.erosion_size =
                this->get_parameter("filter_params.ebus.erosion_size").as_int();
            params.blur_size =
                this->get_parameter("filter_params.ebus.blur_size").as_int();
            params.mask_weight =
                this->get_parameter("filter_params.ebus.mask_weight").as_int();

            filter_ptr = std::make_unique<Ebus>(params);
            break;
        }

        case FilterType::Otsu: {
            OtsuSegmentationParams params;
            params.gamma_auto_correction =
                this->get_parameter("filter_params.otsu.gamma_auto_correction")
                    .as_bool();
            params.gamma_auto_correction_weight =
                this->get_parameter(
                        "filter_params.otsu.gamma_auto_correction_weight")
                    .as_double();
            params.otsu_segmentation =
                this->get_parameter("filter_params.otsu.otsu_segmentation")
                    .as_bool();
            params.gsc_weight_r =
                this->get_parameter("filter_params.otsu.gsc_weight_r")
                    .as_double();
            params.gsc_weight_g =
                this->get_parameter("filter_params.otsu.gsc_weight_g")
                    .as_double();
            params.gsc_weight_b =
                this->get_parameter("filter_params.otsu.gsc_weight_b")
                    .as_double();
            params.erosion_size =
                this->get_parameter("filter_params.otsu.erosion_size").as_int();
            params.dilation_size =
                this->get_parameter("filter_params.otsu.dilation_size")
                    .as_int();

            filter_ptr = std::make_unique<OtsuSegmentation>(params);
            break;
        }

        case FilterType::Overlap: {
            OverlapParams params;
            params.percentage_threshold =
                this->get_parameter(
                        "filter_params.overlap.percentage_threshold")
                    .as_double();

            filter_ptr = std::make_unique<Overlap>(params);
            break;
        }

        case FilterType::MedianBinary: {
            MedianBinaryParams params;
            params.kernel_size =
                this->get_parameter("filter_params.median_binary.kernel_size")
                    .as_int();
            params.threshold =
                this->get_parameter("filter_params.median_binary.threshold")
                    .as_int();
            params.invert =
                this->get_parameter("filter_params.median_binary.invert")
                    .as_bool();

            filter_ptr = std::make_unique<MedianBinary>(params);
            break;
        }

        case FilterType::Binary: {
            BinaryThresholdParams params;
            params.threshold =
                this->get_parameter("filter_params.binary.threshold")
                    .as_double();
            params.maxval =
                this->get_parameter("filter_params.binary.maxval").as_double();
            params.invert =
                this->get_parameter("filter_params.binary.invert").as_bool();

            filter_ptr = std::make_unique<BinaryThreshold>(params);
            break;
        }

        // TODO(New filter): Add your filter case here:
        case FilterType::Example: {
            ExampleParams params;
            params.example_int =
                this->get_parameter("filter_params.example.example_int")
                    .as_int();
            params.example_string =
                this->get_parameter("filter_params.example.example_string")
                    .as_string();

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

void ImageFilteringNode::check_and_subscribe_to_image_topic() {
    std::string image_topic = this->get_parameter("sub_topic").as_string();
    if (image_topic_ != image_topic) {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos_sensor_data = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, qos_sensor_data,
            std::bind(&ImageFilteringNode::image_callback, this, _1));
        image_topic_ = image_topic;
        spdlog::info("Subscribed to image topic: {}", image_topic);
    }
}

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
