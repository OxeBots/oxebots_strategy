#include "oxebots_strategy/kick_ball_node.h"

namespace oxebots_strategy
{

KickBallNode::KickBallNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  // CORREÇÃO: Publicar no tópico correto /robot_commands
  cmd_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", cmd_qos);
  RCLCPP_INFO(node_->get_logger(), "KickBallNode configurado.");
}

BT::PortsList KickBallNode::providedPorts()
{
  // A velocidade de chute padrão foi alterada para 3.0 para corresponder ao log do usuário
  return { BT::InputPort<unsigned int>("robot_id"),
           BT::InputPort<double>("kick_speed", 3.0, "Kick speed in m/s") };
}

BT::NodeStatus KickBallNode::onStart()
{
  unsigned int robot_id;
  double kick_speed;

  if (!getInput<unsigned int>("robot_id", robot_id)) return BT::NodeStatus::FAILURE;
  if (!getInput<double>("kick_speed", kick_speed)) return BT::NodeStatus::FAILURE;

  if (kick_speed > 3.0) {
      kick_speed = 3.0;
  }

  auto msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
  oxebots_interfaces::msg::RobotCmdData robot_cmd_data;
  robot_cmd_data.id = robot_id;
  robot_cmd_data.kick_speed = kick_speed;
  
  // AVANÇAR enquanto chuta para garantir contato
  robot_cmd_data.x_velocity = 0.5; 
  robot_cmd_data.y_velocity = 0.0;
  robot_cmd_data.angular_velocity = 0.0;

  msg->robots.push_back(robot_cmd_data);
  cmd_pub_->publish(std::move(msg));

  RCLCPP_INFO(node_->get_logger(), "Robot %d: CHUTANDO! (vel: %.1f)", robot_id, kick_speed);
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus KickBallNode::onRunning()
{
  // Pausa para dar tempo ao simulador e evitar o loop infinito
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_);
  if (elapsed.count() > 200) {
    RCLCPP_INFO(node_->get_logger(), "KickBallNode finalizado.");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void KickBallNode::onHalted()
{
  // Opcional: Enviar um comando para parar o chute se a ação for cancelada
  unsigned int robot_id;
  if (getInput<unsigned int>("robot_id", robot_id)) {
    auto msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
    oxebots_interfaces::msg::RobotCmdData robot_cmd_data;
    robot_cmd_data.id = robot_id;
    robot_cmd_data.kick_speed = 0.0; // Comando para parar de chutar
    msg->robots.push_back(robot_cmd_data);
    cmd_pub_->publish(std::move(msg));
    RCLCPP_WARN(node_->get_logger(), "KickBallNode (Robot %d) interrompido, chute cancelado.", robot_id);
  }
}

} // namespace oxebots_strategy
