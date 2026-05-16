#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"

namespace oxebots_strategy
{

class PositionBisectorNode : public BT::StatefulActionNode
{
public:
  PositionBisectorNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_pub_;


  uint32_t robot_id_;
  double my_goal_x_;
  
  // Storage for last published goal to avoid spamming the planner
  double last_target_x_ = -99999.0;
  double last_target_y_ = -99999.0;
  double last_target_w_ = -99999.0;
  };

} 