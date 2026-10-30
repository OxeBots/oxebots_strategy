#ifndef OXEBOTS_STRATEGY__PRECISION_KICK_NODE_H_
#define OXEBOTS_STRATEGY__PRECISION_KICK_NODE_H_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_motion_override.hpp"
#include "oxebots_interfaces/msg/robot_motion_status.hpp"
#include <cstdint>

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
  BT::NodeStatus step();
  void statusCallback(const oxebots_interfaces::msg::RobotMotionStatus::SharedPtr msg);
  void releaseOverride();
  double normalize_angle(double angle);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotMotionOverride>::SharedPtr override_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::RobotMotionStatus>::SharedPtr status_sub_;
  oxebots_interfaces::msg::RobotMotionStatus::SharedPtr last_status_;

  unsigned int robot_id_ = 0;
  uint32_t baseline_kick_count_ = 0;
  bool aligned_latched_ = false;
};

} // namespace oxebots_strategy

#endif // OXEBOTS_STRATEGY__PRECISION_KICK_NODE_H_
