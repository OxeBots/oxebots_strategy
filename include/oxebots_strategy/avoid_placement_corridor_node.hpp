#pragma once

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"

namespace oxebots_strategy
{

class AvoidPlacementCorridorNode : public BT::StatefulActionNode
{
public:
  AvoidPlacementCorridorNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus update();
  bool readInputs(
    uint32_t & robot_id,
    double & robot_x,
    double & robot_y,
    double & robot_yaw,
    double & ball_x,
    double & ball_y,
    double & designated_x,
    double & designated_y,
    bool & has_designated,
    double & margin_mm,
    double & max_speed);
  bool publishGoal(
    uint32_t robot_id,
    double target_x,
    double target_y,
    double target_yaw,
    double max_speed);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;
};

}  // namespace oxebots_strategy
