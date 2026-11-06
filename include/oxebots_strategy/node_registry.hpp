#ifndef OXEBOTS_STRATEGY__NODE_REGISTRY_HPP_
#define OXEBOTS_STRATEGY__NODE_REGISTRY_HPP_

#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "oxebots_interfaces/msg/robot_motion_override.hpp"
#include "rclcpp/rclcpp.hpp"

namespace oxebots_strategy
{

void registerStrategyNodes(
  BT::BehaviorTreeFactory & factory,
  const rclcpp::Node::SharedPtr & node,
  const BT::Blackboard::Ptr & blackboard = nullptr,
  const rclcpp::Publisher<oxebots_interfaces::msg::RobotMotionOverride>::SharedPtr &
    override_pub = nullptr,
  int robot_id = 0);

std::string generateTreeNodesModelXML(
  const BT::BehaviorTreeFactory & factory,
  bool include_builtin = false);

}  // namespace oxebots_strategy

#endif  // OXEBOTS_STRATEGY__NODE_REGISTRY_HPP_
