#ifndef OXEBOTS_STRATEGY__GOAL_LINE_DEFEND_NODE_H_
#define OXEBOTS_STRATEGY__GOAL_LINE_DEFEND_NODE_H_

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"

namespace oxebots_strategy
{

class GoalLineDefendNode : public BT::StatefulActionNode
{
public:
  GoalLineDefendNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;

  uint32_t robot_id_;
  double my_goal_x_;
  double last_target_y_ = -99999.0;
};

} // namespace oxebots_strategy

#endif // OXEBOTS_STRATEGY__GOAL_LINE_DEFEND_NODE_H_
