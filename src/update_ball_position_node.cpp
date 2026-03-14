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
  return { BT::OutputPort<double>("ball_x"), 
           BT::OutputPort<double>("ball_y"),
           BT::OutputPort<double>("pre_kick_x"),
           BT::OutputPort<double>("pre_kick_y"),
           BT::InputPort<double>("goal_x"),
           BT::InputPort<double>("goal_y") };
}

void UpdateBallPositionNode::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_game_data_ = msg;
}

BT::NodeStatus UpdateBallPositionNode::onStart()
{
  return onRunning();
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

  // --- Calcular Ponto Atrás da Bola (Pre-Kick) ---
  double goal_x, goal_y;
  if (getInput<double>("goal_x", goal_x) && getInput<double>("goal_y", goal_y)) {
    // Vetor do Gol para a Bola
    double dx = ball_x - goal_x;
    double dy = ball_y - goal_y;
    double dist = std::hypot(dx, dy);

    if (dist > 10.0) {
      // Ponto a 400mm da bola, na mesma linha do gol
      double pre_kick_dist = 400.0; 
      double pk_x = ball_x + (dx / dist) * pre_kick_dist;
      double pk_y = ball_y + (dy / dist) * pre_kick_dist;
      
      setOutput("pre_kick_x", pk_x);
      setOutput("pre_kick_y", pk_y);
    }
  }
  
  return BT::NodeStatus::SUCCESS;
}

void UpdateBallPositionNode::onHalted()
{
  // Nada a fazer aqui
}

} // namespace oxebots_strategy
