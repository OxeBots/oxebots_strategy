#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/robot_game_data.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"
#include <chrono>
#include <cmath>

namespace oxebots_strategy
{

class GoToPointNode : public BT::StatefulActionNode
{
public:
  GoToPointNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  void setRobotId(unsigned int robot_ID) {robot_id_= robot_ID;}
  unsigned int getRobotId() {return robot_id_;} 
  void setTarget(double vel_x, double vel_y, double angle_vel) {target_vel_x_ = vel_x; target_vel_y_ = vel_y;  target_angle_vel_ = angle_vel;}
  void setCurrentPosition(double vel_x_current, double vel_y_current, double angle_vel_current){current_vel_x_ = vel_x_current; current_vel_y_ = vel_y_current; current_angle_vel_ = angle_vel_current; has_current_state_ = true;}

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr publisher_;
  rclcpp::Subscription<oxebots_interfaces::msg::RobotPosition>::SharedPtr robot_state_sub_; // Subscriber para a posição do robô
  
  unsigned int robot_id_;
  double target_vel_x_, target_vel_y_, target_angle_vel_;
  double current_vel_x_, current_vel_y_, current_angle_vel_; // Posição atual do robô
  bool has_current_state_ = false; // Flag para garantir que temos a posição do robô

  // Callback do subscriber
  void robotStateCallback(const oxebots_interfaces::msg::RobotPosition::SharedPtr msg);

};

} // namespace oxebots_strategy