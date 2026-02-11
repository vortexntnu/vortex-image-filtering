
#include <rclcpp_components/register_node_macro.hpp>
#include <ros/image_filtering_ros.hpp>
#include <ros/image_filtering_ros_utils.hpp>
using std::placeholders::_1;
namespace vortex::image_filtering {
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

void ImageFilteringNode::check_and_publish_to_output_topic() {
    std::string pub_topic = this->get_parameter("pub_topic").as_string();
    if (pub_topic_ != pub_topic) {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos_sensor_data = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            pub_topic, qos_sensor_data);

        pub_topic_ = pub_topic;
        spdlog::info("Publishing to image output topic: {}", pub_topic);
    }
}

void ImageFilteringNode::declare_common_ros_params() {
    this->declare_parameter<std::string>("sub_topic");
    this->declare_parameter<std::string>("pub_topic");
    this->declare_parameter<std::string>("output_encoding");
    this->declare_parameter<std::string>("input_encoding");
    this->declare_parameter<std::string>("filter_params.filter_type");
}
}  // namespace vortex::image_filtering
