#ifndef OXEBOTS_STRATEGY__SMOTHER_SAVE_NODE_H_
#define OXEBOTS_STRATEGY__SMOTHER_SAVE_NODE_H_

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"

namespace oxebots_strategy
{

class SmotherSaveNode : public BT::StatefulActionNode
{
public:
  SmotherSaveNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;
  uint32_t robot_id_;
};

}  // namespace oxebots_strategy

#endif  // OXEBOTS_STRATEGY__SMOTHER_SAVE_NODE_H_
