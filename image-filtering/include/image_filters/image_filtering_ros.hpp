#ifndef IMAGE_FILTERING_ROS_HPP
#define IMAGE_FILTERING_ROS_HPP

#include <cv_bridge/cv_bridge.h> // for jazzy this has to be hpp and for humble it has to be h
#include <spdlog/spdlog.h>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "image_processing.hpp"

class ImageFilteringNode : public rclcpp::Node {
   public:
    explicit ImageFilteringNode(const rclcpp::NodeOptions& options);
    ~ImageFilteringNode() {};

   private:
    /**
     * @brief Subscribes to image topic
     */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    /**
     * @brief Publishes the filtered image
     */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    /**
     * @brief Check and subscribe to image if not yet subscribed. Allows for
     * dynamic reconfiguration of image topic. If a new topic is set, the old
     * subscription is cancelled and a new one is bound to the callback
     * function.
     *
     */
    void check_and_subscribe_to_image_topic();

    /**
     * @brief Declare the ros2 parameters used by the node.
     *
     */
    void declare_parameters();

    /**
     * @brief Set the filter parameters for the FilterParams struct.
     *
     */
    void set_filter_params();

    /**
     * @brief Initialize the parameter handler and a parameter event callback.
     *
     */
    void initialize_parameter_handler();
    /**
     * @brief Callback function for parameter events.
     * Checks for parameter changes that matches the nodes' namespace and
     * invokes the relevant initializer functions to update member variables.
     *
     * @param event The parameter event.
     */
    void on_parameter_event(const rcl_interfaces::msg::ParameterEvent& event);

    /**
     * @brief Manages parameter events for the node.
     *
     * This handle is used to set up a mechanism to listen for and react to
     * changes in parameters. Parameters can be used to configure the node's
     * operational behavior dynamically, allowing adjustments without altering
     * the code. The `param_handler_` is responsible for registering callbacks
     * that are triggered on parameter changes, providing a centralized
     * management system within the node for such events.
     */
    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;

    /**
     * @brief Handle to the registration of the parameter event callback.
     *
     * Represents a token or reference to the specific callback registration
     * made with the parameter event handler (`param_handler_`). This handle
     * allows for management of the lifecycle of the callback, such as removing
     * the callback if it's no longer needed. It ensures that the node can
     * respond to parameter changes with the registered callback in an efficient
     * and controlled manner.
     */
    rclcpp::ParameterEventCallbackHandle::SharedPtr param_cb_handle_;

    /**
     * @brief Callback function for image topic
     *
     * @param msg The image message
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief The image topic to subscribe to
     *
     */
    std::string image_topic_;

    /**
     * @brief Pointer to the filter object
     *
     */
    std::unique_ptr<Filter> filter_ptr;

};

#endif  // IMAGE_FILTERING_ROS_HPP
