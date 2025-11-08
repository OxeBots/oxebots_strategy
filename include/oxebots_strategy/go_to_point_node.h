#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include <optional>

namespace oxebots_strategy
{

class GoToPointNode : public BT::StatefulActionNode
{
public:
  GoToPointNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
  std::optional<oxebots_interfaces::msg::RobotGameData> getRobotData(unsigned int robot_id);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;

  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  unsigned int robot_id_;
  geometry_msgs::msg::Point target_pos_;
  double target_w_;
};

} // namespace oxebots_strategy
