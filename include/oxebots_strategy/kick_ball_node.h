#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"

namespace oxebots_strategy
{

class KickBallNode : public BT::StatefulActionNode
{
public:
  KickBallNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_pub_;
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

} // namespace oxebots_strategy
