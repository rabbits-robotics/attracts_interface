#include <rclcpp/rclcpp.hpp>

#include "attracts_interface/game_client_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GameClient>());
  rclcpp::shutdown();
  return 0;
}
