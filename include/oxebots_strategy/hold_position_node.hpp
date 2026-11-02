#pragma once

#include <mutex>
#include <string>
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"

namespace oxebots_strategy
{

class HoldPositionNode : public BT::StatefulActionNode
{
public:
  HoldPositionNode(const std::string& name, const BT::NodeConfig& config,
                   rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
  bool publishHere();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;

  std::mutex mutex_;
  oxebots_interfaces::msg::GameData::SharedPtr last_data_;
  unsigned int robot_id_{0};
};

}  // namespace oxebots_strategy
