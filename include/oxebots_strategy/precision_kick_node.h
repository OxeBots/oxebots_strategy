#ifndef OXEBOTS_STRATEGY__PRECISION_KICK_NODE_H_
#define OXEBOTS_STRATEGY__PRECISION_KICK_NODE_H_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/robot_cmd_data.hpp"
#include <chrono>

namespace oxebots_strategy
{

class PrecisionKickNode : public BT::StatefulActionNode
{
public:
  PrecisionKickNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void send_command(double vx, double vy, double w, double kick_speed);
  double normalize_angle(double angle);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_pub_;
  
  std::chrono::steady_clock::time_point start_time_;
  bool kick_triggered_ = false;
};

} // namespace oxebots_strategy

#endif // OXEBOTS_STRATEGY__PRECISION_KICK_NODE_H_
