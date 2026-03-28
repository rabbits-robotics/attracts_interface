#include <gtest/gtest.h>

#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "attracts_interface/game_client_node.hpp"

class GameClientTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("max_omni_vel", 1.0),
      rclcpp::Parameter("max_omni_rot_vel", 2.0),
      rclcpp::Parameter("max_yaw_rot_vel", 3.0),
      rclcpp::Parameter("max_pitch_rot_vel", 3.0),
      });
    node_ = std::make_shared<GameClient>(options);
  }

  void TearDown() override
  {
    node_.reset();
  }

  std::shared_ptr<GameClient> node_;
};

static attracts_msgs::msg::GameDataInput MakeInput()
{
  return attracts_msgs::msg::GameDataInput();
}

TEST_F(GameClientTest, KeyWSetsChassisFwd)
{
  auto input = MakeInput();
  input.key_w = true;
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(1.0, cmd.chassis_vel.x);
}

TEST_F(GameClientTest, KeySSetsChassisBwd)
{
  auto input = MakeInput();
  input.key_s = true;
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(-1.0, cmd.chassis_vel.x);
}

TEST_F(GameClientTest, KeyASetsChassisLeft)
{
  auto input = MakeInput();
  input.key_a = true;
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(1.0, cmd.chassis_vel.y);
}

TEST_F(GameClientTest, KeyDSetsChassisRight)
{
  auto input = MakeInput();
  input.key_d = true;
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(-1.0, cmd.chassis_vel.y);
}

TEST_F(GameClientTest, KeyZSetsChassisRotCcw)
{
  auto input = MakeInput();
  input.key_z = true;
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(2.0, cmd.chassis_vel.z);
}

TEST_F(GameClientTest, KeyCSetsChassisRotCw)
{
  auto input = MakeInput();
  input.key_c = true;
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(-2.0, cmd.chassis_vel.z);
}

TEST_F(GameClientTest, MouseDeltaXSetsYaw)
{
  auto input = MakeInput();
  input.mouse_delta_x = 100;  // yaw_pos = 0 + 100/100 = 1.0
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(1.0, cmd.yaw_pos);
}

TEST_F(GameClientTest, YawWrapsNegative)
{
  auto input = MakeInput();
  input.mouse_delta_x = -1000;  // yaw = -10, fmod → negative → add 2π
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_GE(cmd.yaw_pos, 0.0);
}

TEST_F(GameClientTest, PitchClampedLow)
{
  auto input = MakeInput();
  input.mouse_delta_y = -1000;  // pitch = -10 → clamped to -M_PI/12
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_NEAR(-M_PI / 12, cmd.pitch_pos, 1e-6);
}

TEST_F(GameClientTest, PitchClampedHigh)
{
  auto input = MakeInput();
  input.mouse_delta_y = 1000;  // pitch = 10 → clamped to M_PI/6
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_NEAR(M_PI / 6, cmd.pitch_pos, 1e-6);
}

TEST_F(GameClientTest, MouseRightButtonSetsFire)
{
  auto input = MakeInput();
  input.mouse_right_button = true;
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(1, cmd.fire_mode);
}

TEST_F(GameClientTest, MouseLeftButtonSetsLoad)
{
  auto input = MakeInput();
  input.mouse_left_button = true;
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(1, cmd.load_mode);
}

TEST_F(GameClientTest, KeyRSetsLoad2)
{
  auto input = MakeInput();
  input.key_r = true;
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(2, cmd.load_mode);
}

TEST_F(GameClientTest, KeyShiftSetsChassisModeOn)
{
  auto input = MakeInput();
  input.key_shift = true;
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(1, cmd.chassis_mode);
}

TEST_F(GameClientTest, SpeedModeAlwaysZero)
{
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(0, cmd.speed_mode);
}

TEST_F(GameClientTest, UpdatePositionsRuns)
{
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_NO_THROW(node_->UpdatePositions(cmd));
}

TEST_F(GameClientTest, GameDataInputCBSetsMsg)
{
  auto input = MakeInput();
  input.key_w = true;
  node_->GameDataInputCB(std::make_shared<attracts_msgs::msg::GameDataInput>(input));
  attracts_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(1.0, cmd.chassis_vel.x);
}
