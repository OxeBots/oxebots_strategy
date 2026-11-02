#pragma once

#include <string>
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace oxebots_strategy
{

class SetMotionLimitNode : public BT::SyncActionNode
{
public:
  SetMotionLimitNode(const std::string& name, const BT::NodeConfig& config,
                     rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr limit_pub_;
};

}  // namespace oxebots_strategy
