#pragma once

#include <string>
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace oxebots_strategy
{

class ComputeLegalSlotNode : public BT::SyncActionNode
{
public:
  ComputeLegalSlotNode(const std::string& name, const BT::NodeConfig& config,
                       rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace oxebots_strategy
