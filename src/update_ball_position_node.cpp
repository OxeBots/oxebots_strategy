#include "oxebots_strategy/update_ball_position_node.h"

namespace oxebots_strategy
{

UpdateBallPositionNode::UpdateBallPositionNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  // Subscriber para receber os dados do jogo (visão e estado)
  game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10, std::bind(&UpdateBallPositionNode::gameDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "UpdateBallPositionNode configurado.");
}

// Definição das portas de entrada e saída do nó no Behavior Tree
BT::PortsList UpdateBallPositionNode::providedPorts()
{
  return { BT::OutputPort<double>("ball_x"), 
           BT::OutputPort<double>("ball_y"),
           BT::OutputPort<double>("pre_kick_x"),
           BT::OutputPort<double>("pre_kick_y"),
           BT::OutputPort<bool>("is_ready_to_kick"),
           BT::InputPort<unsigned int>("robot_id"),
           BT::InputPort<double>("goal_x"),
           BT::InputPort<double>("goal_y") };
}

void UpdateBallPositionNode::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  // Protege o acesso aos dados com um mutex, já que o callback roda em outra thread
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_game_data_ = msg;
}

BT::NodeStatus UpdateBallPositionNode::onStart()
{
  // No início, apenas chama o onRunning para processar os dados imediatamente
  return onRunning();
}

BT::NodeStatus UpdateBallPositionNode::onRunning()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // Se ainda não recebemos dados da visão, avisa e continua esperando
  if (!last_game_data_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Nenhum dado de jogo (last_game_data) recebido, aguardando...");
    return BT::NodeStatus::RUNNING;
  }
  
  // Extrai a posição atual da bola e envia para as portas de saída
  double ball_x = last_game_data_->ball.x;
  double ball_y = last_game_data_->ball.y;

  setOutput("ball_x", ball_x);
  setOutput("ball_y", ball_y);

  // --- Calcular Ponto Atrás da Bola (Pre-Kick) ---
  // Este ponto é onde o robô deve se posicionar para chutar a bola em direção ao gol
  double goal_x, goal_y;
  unsigned int robot_id;
  if (getInput<double>("goal_x", goal_x) && getInput<double>("goal_y", goal_y)) {
    // Vetor que aponta do Gol para a Bola
    double dx = ball_x - goal_x;
    double dy = ball_y - goal_y;
    double dist_ball_goal = std::hypot(dx, dy);

    if (dist_ball_goal > 10.0) {
      // Define um ponto a 400mm da bola, seguindo a linha reta que vem do gol
      double pre_kick_dist = 400.0; 
      double pk_x = ball_x + (dx / dist_ball_goal) * pre_kick_dist;
      double pk_y = ball_y + (dy / dist_ball_goal) * pre_kick_dist;
      
      setOutput("pre_kick_x", pk_x);
      setOutput("pre_kick_y", pk_y);

      // --- Verificar se o robô já está "atrás da bola" ---
      if (getInput<unsigned int>("robot_id", robot_id)) {
        for (const auto& ally : last_game_data_->robots.allies) {
          if (ally.id == robot_id) {
            double rx = ally.x;
            double ry = ally.y;

            // Vetor do robô para a bola
            double r_dx = ball_x - rx;
            double r_dy = ball_y - ry;
            double dist_robot_ball = std::hypot(r_dx, r_dy);

            // Produto escalar para verificar se o robô está do lado oposto ao gol em relação à bola
            // Isso garante que ele não tente chutar "atravessando" a bola pelo lado errado
            double dot = (dx * r_dx + dy * r_dy);
            
            // Se o robô estiver a menos de 500mm e bem posicionado (ângulo favorável), está pronto
            bool is_ready = (dist_robot_ball < 500.0 && dot > 0);
            setOutput("is_ready_to_kick", is_ready);
            break;
          }
        }
      }
    }
  }
  
  // Este nó sempre retorna SUCCESS pois sua função é apenas atualizar os dados no Blackboard
  return BT::NodeStatus::SUCCESS;
}

void UpdateBallPositionNode::onHalted()
{
  // Nada a fazer aqui
}

} // namespace oxebots_strategy
