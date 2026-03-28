#include <rclcpp/rclcpp.hpp>

#include "attracts_interface/gamepad_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Gamepad>());
  rclcpp::shutdown();
  return 0;
}
