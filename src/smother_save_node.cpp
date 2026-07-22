#include "oxebots_strategy/smother_save_node.h"
#include <cmath>

namespace oxebots_strategy
{

SmotherSaveNode::SmotherSaveNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_(node_ptr)
{
  auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);
}

BT::PortsList SmotherSaveNode::providedPorts()
{
  return {
    BT::InputPort<unsigned int>("robot_id"),
    BT::InputPort<double>("ball_x"),
    BT::InputPort<double>("ball_y")
  };
}

BT::NodeStatus SmotherSaveNode::onStart()
{
  // Lido em onStart() (não no construtor): portas remapeadas por blackboard só têm valor
  // garantido a partir daqui, já que o nó é construído durante a montagem da árvore.
  if (!getInput<unsigned int>("robot_id", robot_id_)) {
    RCLCPP_ERROR(node_->get_logger(), "SmotherSaveNode: Erro: porta [robot_id] ausente");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_WARN(node_->get_logger(), "Goleiro %d: ABAFANDO A BOLA (Smother Save)!", robot_id_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SmotherSaveNode::onRunning()
{
  double ball_x, ball_y;
  if (!getInput<double>("ball_x", ball_x) || !getInput<double>("ball_y", ball_y)) {
    return BT::NodeStatus::FAILURE;
  }

  auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
  goal_msg->robot_id = robot_id_;
  goal_msg->pose.header.stamp = node_->now();
  goal_msg->pose.header.frame_id = "map";

  // Avança em direção à bola
  goal_msg->pose.pose.position.x = ball_x / 1000.0;
  goal_msg->pose.pose.position.y = ball_y / 1000.0;
  goal_msg->pose.pose.orientation.w = 1.0; 

  goal_pub_->publish(std::move(goal_msg));

  return BT::NodeStatus::RUNNING;
}

void SmotherSaveNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "SmotherSaveNode abortado.");
}

} // namespace oxebots_strategy
