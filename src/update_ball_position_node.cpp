#include "oxebots_strategy/update_ball_position_node.h"

namespace oxebots_strategy
{

UpdateBallPositionNode::UpdateBallPositionNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10, std::bind(&UpdateBallPositionNode::gameDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "UpdateBallPositionNode configurado.");
}


BT::PortsList UpdateBallPositionNode::providedPorts()
{
  return { BT::OutputPort<double>("ball_x"), BT::OutputPort<double>("ball_y") };
}

void UpdateBallPositionNode::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_game_data_ = msg;
}

BT::NodeStatus UpdateBallPositionNode::onStart()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!last_game_data_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Nenhum dado de jogo (last_game_data) recebido, aguardando...");
    return BT::NodeStatus::RUNNING;
  }

  double ball_x = last_game_data_->ball.x;
  double ball_y = last_game_data_->ball.y;

  setOutput("ball_x", ball_x);
  setOutput("ball_y", ball_y);
  
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UpdateBallPositionNode::onRunning()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!last_game_data_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Nenhum dado de jogo (last_game_data) recebido, aguardando...");
    return BT::NodeStatus::RUNNING;
  }
  
  double ball_x = last_game_data_->ball.x;
  double ball_y = last_game_data_->ball.y;

  setOutput("ball_x", ball_x);
  setOutput("ball_y", ball_y);
  
  return BT::NodeStatus::SUCCESS;
}

void UpdateBallPositionNode::onHalted()
{
  // Nada a fazer aqui
}

} // namespace oxebots_strategy
