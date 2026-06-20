#ifndef ATTRACTS_INTERFACE__GAME_CLIENT_NODE_HPP_
#define ATTRACTS_INTERFACE__GAME_CLIENT_NODE_HPP_

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <attracts_msgs/msg/attracts_command.hpp>
#include <attracts_msgs/msg/game_data_input.hpp>
#include <attracts_msgs/msg/game_data_robot.hpp>

class GameClient : public rclcpp::Node
{
public:
  explicit GameClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void GameDataInputCB(const attracts_msgs::msg::GameDataInput::SharedPtr msg);
  void GameDataRobotCB(const attracts_msgs::msg::GameDataRobot::SharedPtr msg);
  void TimerCB();
  void UpdateCmdVel(attracts_msgs::msg::AttractsCommand & cmd);
  void UpdatePositions(const attracts_msgs::msg::AttractsCommand & cmd);

  // 次の射撃で熱量が最大に達する場合は装填を禁止する（オーバーヒート防止）。
  // 熱量データ未受信（max_heat == 0）のときはリミットを適用しない。
  bool CanLoad() const;

private:
  static constexpr double TIMER_FREQ = 100.0;  // Hz

  rclcpp::Publisher<attracts_msgs::msg::AttractsCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<attracts_msgs::msg::GameDataInput>::SharedPtr game_data_input_sub_;
  rclcpp::Subscription<attracts_msgs::msg::GameDataRobot>::SharedPtr game_data_robot_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double wheel_d_ = 0.06;  // m
  double body_d_ = 0.28;   // m
  double max_omni_vel_;     // m/s
  double max_omni_rot_vel_;  // rad/s
  double max_yaw_rot_vel_;   // rad/s
  double max_pitch_rot_vel_;  // rad/s
  double fire_heat_;  // 1射あたりの熱量増加量

  attracts_msgs::msg::GameDataInput game_data_input_msg_;
  attracts_msgs::msg::GameDataRobot game_data_robot_msg_;
  std::array<double, 6> positions_;
};

#endif  // ATTRACTS_INTERFACE__GAME_CLIENT_NODE_HPP_
