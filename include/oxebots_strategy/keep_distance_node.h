#ifndef OXEBOTS_STRATEGY__KEEP_DISTANCE_NODE_H_
#define OXEBOTS_STRATEGY__KEEP_DISTANCE_NODE_H_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"

namespace oxebots_strategy
{
class KeepDistanceNode : public BT::StatefulActionNode
{
public:
  KeepDistanceNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_pub_;
};
}  // namespace oxebots_strategy

#endif  // OXEBOTS_STRATEGY__KEEP_DISTANCE_NODE_H_