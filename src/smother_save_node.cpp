#include "oxebots_strategy/smother_save_node.h"
#include <cmath>
#include <memory>


namespace oxebots_strategy
{


SmotherSaveNode::SmotherSaveNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr)
 : BT::StatefulActionNode(name, config), node_(node_ptr)
{
 if (!getInput<int>("robot_id", robot_id_)) {
     RCLCPP_ERROR(node_->get_logger(), "Erro: porta [robot_id] ausente");
 }


 goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", 10);
 cmd_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", 10);
}


BT::PortsList SmotherSaveNode::providedPorts()
{
 return {
     BT::InputPort<int>("robot_id"),
     BT::InputPort<double>("ball_x"),
     BT::InputPort<double>("ball_y")
 };
}


BT::NodeStatus SmotherSaveNode::onStart()
{
 RCLCPP_WARN(node_->get_logger(), "Goleiro %d: BOTE FRONTAL ATIVADO!", robot_id_);
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
 goal_msg->pose.pose.position.x = ball_x / 1000.0;
 goal_msg->pose.pose.position.y = ball_y / 1000.0;


 double target_w = std::atan2(-ball_y, -ball_x);
 goal_msg->pose.pose.orientation.z = std::sin(target_w * 0.5);
 goal_msg->pose.pose.orientation.w = std::cos(target_w * 0.5);
 goal_pub_->publish(std::move(goal_msg));


 auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
 oxebots_interfaces::msg::RobotCmdData cmd_data;
 cmd_data.id = robot_id_;
 cmd_data.kick_speed = 5.0;
 cmd_msg->robots.push_back(cmd_data);
 cmd_pub_->publish(std::move(cmd_msg));


 return BT::NodeStatus::RUNNING;
}


void SmotherSaveNode::onHalted()
{
 auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
 oxebots_interfaces::msg::RobotCmdData cmd_data;
 cmd_data.id = robot_id_;
 cmd_data.kick_speed = 0.0;
 cmd_msg->robots.push_back(cmd_data);
 cmd_pub_->publish(std::move(cmd_msg));
}


}
