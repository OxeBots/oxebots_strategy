#include "oxebots_strategy/precision_kick_node.h"
#include <cmath>

namespace oxebots_strategy
{

PrecisionKickNode::PrecisionKickNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  cmd_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", cmd_qos);
}

BT::PortsList PrecisionKickNode::providedPorts()
{
  return { BT::InputPort<unsigned int>("robot_id", "ID do robo"),
           BT::InputPort<double>("target_x", "X do alvo"),
           BT::InputPort<double>("target_y", "Y do alvo"),
           BT::InputPort<double>("kick_speed", 3.5, "Velocidade do chute") };
}

BT::NodeStatus PrecisionKickNode::onStart()
{
  kick_triggered_ = false;
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PrecisionKickNode::onRunning()
{
  unsigned int robot_id;
  double target_x, target_y, kick_speed;
  double rx, ry, ryaw, bx, by;

  // Obter entradas
  if (!getInput<unsigned int>("robot_id", robot_id)) return BT::NodeStatus::FAILURE;
  if (!getInput<double>("target_x", target_x)) return BT::NodeStatus::FAILURE;
  if (!getInput<double>("target_y", target_y)) return BT::NodeStatus::FAILURE;
  if (!getInput<double>("kick_speed", kick_speed)) kick_speed = 3.5;

  // Obter dados do Blackboard (em MM ou Metros dependendo do sistema, aqui assumimos a mesma escala do GameData)
  if (!config().blackboard->get("robot_x", rx)) return BT::NodeStatus::FAILURE;
  if (!config().blackboard->get("robot_y", ry)) return BT::NodeStatus::FAILURE;
  if (!config().blackboard->get("robot_yaw", ryaw)) return BT::NodeStatus::FAILURE;
  if (!config().blackboard->get("ball_x", bx)) return BT::NodeStatus::FAILURE;
  if (!config().blackboard->get("ball_y", by)) return BT::NodeStatus::FAILURE;

  // Se o chute já foi disparado, esperamos um pouco para o robô completar o movimento
  if (kick_triggered_) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_);
    if (elapsed.count() > 150) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  // 1. Calcular vetor Bola -> Alvo (direção do chute ideal)
  double dx_kick = target_x - bx;
  double dy_kick = target_y - by;
  double angle_kick = atan2(dy_kick, dx_kick);

  // 2. Erro de orientação atual do robô em relação ao chute ideal
  double angle_error = normalize_angle(angle_kick - ryaw);

  // 3. Vetor Robô -> Bola (distância atual)
  double dx_ball = bx - rx;
  double dy_ball = by - ry;
  double dist_ball = sqrt(dx_ball * dx_ball + dy_ball * dy_ball);

  // 4. Lógica de Controle Suavizada
  double vx = 0.0, vy = 0.0, w = 0.0, k = 0.0;

  // Controle de giro (P-Controller com Deadband)
  if (std::abs(angle_error) > 0.05) { // Só gira se o erro for maior que ~3 graus
      w = angle_error * 2.5; // Reduzi de 4.0 para 2.5 para evitar oscilação
      if (w > 2.0) w = 2.0;
      if (w < -2.0) w = -2.0;
  } else {
      w = 0.0; // Ângulo perfeito
  }

  // Só avança se o ângulo estiver minimamente aceitável
  if (std::abs(angle_error) < 0.4) {
      // Aproximação linear: quanto mais longe, mais rápido (até um limite)
      vx = 0.2 + (dist_ball / 1000.0) * 0.5; 
      if (vx > 0.6) vx = 0.6;
      
      // Se o ângulo estiver MUITO bom e estiver na distância de chute
      if (std::abs(angle_error) < 0.15 && dist_ball < 130.0) { 
          vx = 1.0; // Lunge final agressivo
          k = kick_speed;
          kick_triggered_ = true;
          start_time_ = std::chrono::steady_clock::now();
          RCLCPP_INFO(node_->get_logger(), "PrecisionKick (Robot %d): CHUTE DISPARADO!", robot_id);
      }
  } else {
      // Se estiver muito desalinhado, para e foca em girar
      vx = 0.0;
  }

  send_command(vx, vy, w, k);
  return BT::NodeStatus::RUNNING;
}

void PrecisionKickNode::onHalted()
{
  send_command(0.0, 0.0, 0.0, 0.0);
}

void PrecisionKickNode::send_command(double vx, double vy, double w, double kick_speed)
{
  unsigned int robot_id;
  if (!getInput<unsigned int>("robot_id", robot_id)) return;

  auto msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
  oxebots_interfaces::msg::RobotCmdData data;
  data.id = robot_id;
  data.x_velocity = vx;
  data.y_velocity = vy;
  data.angular_velocity = w;
  data.kick_speed = kick_speed;
  
  msg->robots.push_back(data);
  cmd_pub_->publish(std::move(msg));
}

double PrecisionKickNode::normalize_angle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

} // namespace oxebots_strategy
