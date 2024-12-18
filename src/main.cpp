
#include "lidarfilter_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const rclcpp::NodeOptions options;
  auto node = std::make_shared<Filter::lidarfilter_node>(options);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  //gps2local();
  return 0;
}