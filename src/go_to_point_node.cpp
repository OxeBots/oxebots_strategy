#include "oxebots_strategy/go_to_point_node.h"
#include <cmath>

namespace {
double normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
} // namespace

namespace oxebots_strategy
{ 

GoToPointNode::GoToPointNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);
  
  game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10, std::bind(&GoToPointNode::gameDataCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(node_->get_logger(), "GoToPointNode (com mira fixa) configurado.");
}

BT::PortsList GoToPointNode::providedPorts()
{
  return { BT::InputPort<unsigned int>("robot_id"),
           BT::InputPort<double>("x"),
           BT::InputPort<double>("y") };
}

void GoToPointNode::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  last_game_data_ = msg;
}

std::optional<oxebots_interfaces::msg::RobotGameData> GoToPointNode::getRobotData(unsigned int robot_id) {
    if (!last_game_data_) return std::nullopt;
    for (const auto& ally : last_game_data_->robots.allies) {
        if (ally.id == robot_id) return ally;
    }
    return std::nullopt;
}

BT::NodeStatus GoToPointNode::onStart()
{
  if (!getInput<unsigned int>("robot_id", robot_id_)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing required input [robot_id]");
      return BT::NodeStatus::FAILURE;
  }

  double target_x, target_y;
  if (!getInput<double>("x", target_x) || !getInput<double>("y", target_y)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing required input [x] or [y]");
      return BT::NodeStatus::FAILURE;
  }

  target_pos_.x = target_x;
  target_pos_.y = target_y;

  double opponent_goal_x, opponent_goal_y;
  auto blackboard = config().blackboard;
  if (!blackboard->get("opponent_goal_x", opponent_goal_x) || !blackboard->get("opponent_goal_y", opponent_goal_y)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing opponent goal from blackboard");
      return BT::NodeStatus::FAILURE;
  }
 
  target_w_ = std::atan2(opponent_goal_y - target_pos_.y, opponent_goal_x - target_pos_.x);
  
  auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
  goal_msg->robot_id = robot_id_; 
  
  goal_msg->pose.header.stamp = node_->now();
  goal_msg->pose.header.frame_id = "odom"; 
  goal_msg->pose.pose.position.x = target_pos_.x;
  goal_msg->pose.pose.position.y = target_pos_.y;
  
  goal_msg->pose.pose.orientation.x = 0.0;
  goal_msg->pose.pose.orientation.y = 0.0;
  goal_msg->pose.pose.orientation.z = std::sin(target_w_ * 0.5);
  goal_msg->pose.pose.orientation.w = std::cos(target_w_ * 0.5);

  goal_pub_->publish(std::move(goal_msg));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPointNode::onRunning()
{
  auto robot_data = getRobotData(robot_id_);
  if (!robot_data) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000, 
        "No data for robot %d, cannot check goal condition. Waiting...", robot_id_);
      return BT::NodeStatus::RUNNING; 
  }

  double dist_to_goal = std::hypot(robot_data->x - target_pos_.x, robot_data->y - target_pos_.y);
  bool position_ok = (dist_to_goal < 150.0);

  double current_w = robot_data->orientation; 
  double angle_error = normalizeAngle(target_w_ - current_w);
  bool orientation_ok = (std::abs(angle_error) < 0.1);

  if (position_ok && orientation_ok) {
      return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void GoToPointNode::onHalted()
{
  if (auto robot_data = getRobotData(robot_id_)) {
    auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
    goal_msg->robot_id = robot_id_;
    
    double current_w = robot_data->orientation;

    goal_msg->pose.header.stamp = node_->now();
    goal_msg->pose.header.frame_id = "odom";
    goal_msg->pose.pose.position.x = robot_data->x;
    goal_msg->pose.pose.position.y = robot_data->y;
    
    goal_msg->pose.pose.orientation.x = 0.0;
    goal_msg->pose.pose.orientation.y = 0.0;
    goal_msg->pose.pose.orientation.z = std::sin(current_w * 0.5);
    goal_msg->pose.pose.orientation.w = std::cos(current_w * 0.5);
    
    goal_pub_->publish(std::move(goal_msg));
  }
}

}  // namespace oxebots_strategy
