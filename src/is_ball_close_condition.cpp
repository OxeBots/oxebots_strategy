#include "oxebots_strategy/is_ball_close_condition.h"
#include <cmath>

namespace oxebots_strategy
{

IsBallCloseCondition::IsBallCloseCondition(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node)
{
  game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10, std::bind(&IsBallCloseCondition::gameDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "IsBallCloseCondition configurado.");
}

void IsBallCloseCondition::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_game_data_ = msg;
}

BT::PortsList IsBallCloseCondition::providedPorts()
{
  return { BT::InputPort<unsigned int>("robot_id"),
           BT::InputPort<double>("distance_threshold", 180.0, "Distância para considerar 'perto'") };
}

BT::NodeStatus IsBallCloseCondition::tick()
{
  unsigned int robot_id;
  double threshold;
  if (!getInput<unsigned int>("robot_id", robot_id) || !getInput<double>("distance_threshold", threshold)) {
    RCLCPP_ERROR(node_->get_logger(), "Faltando portas [robot_id] ou [distance_threshold]");
    return BT::NodeStatus::FAILURE;
  }

  double ball_x, ball_y;
  if (!config().blackboard->get("ball_x", ball_x) || !config().blackboard->get("ball_y", ball_y)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Posição da bola não encontrada no blackboard.");
    return BT::NodeStatus::FAILURE;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!last_game_data_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Nenhum dado de jogo recebido.");
    return BT::NodeStatus::FAILURE;
  }

  double robot_x, robot_y;
  bool robot_found = false;
  for (const auto& ally : last_game_data_->robots.allies) {
    if (ally.id == robot_id) {
      robot_x = ally.x;
      robot_y = ally.y;
      robot_found = true;
      break;
    }
  }

  if (!robot_found) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Robô com ID %d não encontrado.", robot_id);
    return BT::NodeStatus::FAILURE;
  }

  double distance = std::hypot(robot_x - ball_x, robot_y - ball_y);

  if (distance < threshold) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "Robô %d está perto da bola (distância: %.1f)", robot_id, distance);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Robô %d está longe da bola (distância: %.1f)", robot_id, distance);
    return BT::NodeStatus::FAILURE;
  }
}

} // namespace oxebots_strategy
