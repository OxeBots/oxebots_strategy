#include "oxebots_strategy/go_to_point_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

namespace oxebots_strategy
{

GoToPointNode::GoToPointNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  
  auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", goal_qos);
  game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10, std::bind(&GoToPointNode::gameDataCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(node_->get_logger(), "GoToPointNode configured to publish goals.");
}

BT::PortsList GoToPointNode::providedPorts()
{
  return { BT::InputPort<unsigned int>("robot_id"),
           BT::InputPort<double>("x"),
           BT::InputPort<double>("y"),
           BT::InputPort<double>("w") };
}

void GoToPointNode::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  last_game_data_ = msg;
  game_data_received_ = true;
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

  // Armazena o alvo
  target_pos_.x = target_x;
  target_pos_.y = target_y;

  // Publica o alvo para o nó de campo potencial
  auto goal_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
  goal_msg->header.stamp = node_->now();
  goal_msg->header.frame_id = "odom"; // Ou o frame apropriado
  goal_msg->pose.position.x = target_pos_.x;
  goal_msg->pose.position.y = target_pos_.y;
  goal_msg->pose.orientation.w = 0.0; // Orientação neutra

  goal_pub_->publish(std::move(goal_msg));

  RCLCPP_INFO(node_->get_logger(), "GoToPointNode: Published new goal (%.2f, %.2f) for robot %d", target_pos_.x, target_pos_.y, robot_id_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPointNode::onRunning()
{
  if (!game_data_received_) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for game data...");
    return BT::NodeStatus::RUNNING;
  }

  auto robot_data = getRobotData(robot_id_);
  if (!robot_data) {
      RCLCPP_WARN(node_->get_logger(), "No data for robot %d, cannot check goal condition.", robot_id_);
      return BT::NodeStatus::RUNNING; // Continua em execução esperando por dados
  }

  // Verifica a condição de sucesso
  double dist_to_goal = std::hypot(robot_data->x - target_pos_.x, robot_data->y - target_pos_.y);

  if (dist_to_goal < 150.0) { // Limiar de 150mm (15cm)
      RCLCPP_INFO(node_->get_logger(), "GoToPointNode: Robot %d reached goal.", robot_id_);
      return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void GoToPointNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "GoToPointNode halted. Commanding robot %d to stop.", robot_id_);
  
  // Publica a posição atual do robô como o novo alvo para fazê-lo parar.
  if (auto robot_data = getRobotData(robot_id_)) {
    auto goal_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    goal_msg->header.stamp = node_->now();
    goal_msg->header.frame_id = "odom";
    goal_msg->pose.position.x = robot_data->x;
    goal_msg->pose.position.y = robot_data->y;
    goal_msg->pose.orientation.w = 1.0;
    goal_pub_->publish(std::move(goal_msg));
  }
}

}  // namespace oxebots_strategy
