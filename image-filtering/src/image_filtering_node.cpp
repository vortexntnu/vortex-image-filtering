#include <image_filters/image_filtering_ros.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<vortex::image_processing::ImageFilteringNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}