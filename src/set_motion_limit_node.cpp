#include "oxebots_strategy/set_motion_limit_node.hpp"
#include <algorithm>
#include <cmath>

namespace oxebots_strategy
{

SetMotionLimitNode::SetMotionLimitNode(const std::string& name, const BT::NodeConfig& config,
                                       rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), node_(node)
{
  if (!node_->has_parameter("absolute_max_mps")) {
    node_->declare_parameter<double>("absolute_max_mps", 1.5);
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  limit_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/robot_motion_limit", qos);
}

BT::PortsList SetMotionLimitNode::providedPorts()
{
  return {
    BT::InputPort<double>("limit_mps", 1.5, "Teto de velocidade linear (m/s) enviado ao SafetyGate")
  };
}

BT::NodeStatus SetMotionLimitNode::tick()
{
  double limit_mps = 1.5;
  if (!getInput<double>("limit_mps", limit_mps) || !std::isfinite(limit_mps) || limit_mps <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "SetMotionLimit: valor inválido na porta limit_mps");
    return BT::NodeStatus::FAILURE;
  }

  double absolute_max_mps = 1.5;
  if (node_->has_parameter("absolute_max_mps")) {
    absolute_max_mps = node_->get_parameter("absolute_max_mps").as_double();
  }

  double effective_limit = std::clamp(limit_mps, 0.0, absolute_max_mps);

  auto msg = std::make_unique<std_msgs::msg::Float32>();
  msg->data = static_cast<float>(effective_limit);
  limit_pub_->publish(std::move(msg));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace oxebots_strategy
