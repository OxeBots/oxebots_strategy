#include "oxebots_strategy/keep_distance_node.h"
#include <cmath>

namespace oxebots_strategy
{
KeepDistanceNode::KeepDistanceNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), ros_node_(node)
{
  cmd_pub_ = ros_node_->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", 10);
}

BT::PortsList KeepDistanceNode::providedPorts()
{
  return {
    BT::InputPort<uint32_t>("robot_id"),
    BT::InputPort<double>("ball_x"),
    BT::InputPort<double>("ball_y"),
    BT::InputPort<double>("distance")
  };
}

BT::NodeStatus KeepDistanceNode::onStart()
{
  return onRunning();
}

BT::NodeStatus KeepDistanceNode::onRunning()
{
  uint32_t robot_id;
  double ball_x, ball_y, min_distance;

  // Lemos as coordenadas da porta XML
  if (!getInput("robot_id", robot_id) || !getInput("ball_x", ball_x) || !getInput("ball_y", ball_y)) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Faltam parametros no KeepDistance");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("distance", min_distance)) {
    min_distance = 550.0; // Padrão: 500mm da regra + 50mm de margem de segurança
  }

  // Lemos a própria posição do robô no Blackboard
  double robot_x, robot_y;
  if (!config().blackboard->get("robot_x", robot_x) || !config().blackboard->get("robot_y", robot_y)) {
    return BT::NodeStatus::FAILURE;
  }

  // Teorema de Pitágoras para calcular a distância
  double dx = robot_x - ball_x;
  double dy = robot_y - ball_y;
  double current_dist = std::hypot(dx, dy);

  auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
  oxebots_interfaces::msg::RobotCmdData cmd_data;
  cmd_data.id = robot_id;
  cmd_data.angular_velocity = 0.0;
  cmd_data.kick_speed = 0.0;

  // Se já está na distância legal, para os motores e retorna sucesso
  if (current_dist >= min_distance) {
    cmd_data.x_velocity = 0.0;
    cmd_data.y_velocity = 0.0;
    cmd_msg->robots.push_back(cmd_data);
    cmd_pub_->publish(std::move(cmd_msg));
    return BT::NodeStatus::SUCCESS;
  }

  // Para evitar divisão por zero se a bola estiver exatamente no centro do robô
  if (current_dist < 1.0) {
    dx = 1.0; dy = 0.0; current_dist = 1.0;
  }

  // Regra Oficial SSL: MAX 0.75 m/s durante STOP
  // Criamos um vetor unitário (dx/dist) e multiplicamos pela velocidade máxima permitida
  double vx = (dx / current_dist) * 0.75;
  double vy = (dy / current_dist) * 0.75;

  cmd_data.x_velocity = vx;
  cmd_data.y_velocity = vy;
  cmd_msg->robots.push_back(cmd_data);
  cmd_pub_->publish(std::move(cmd_msg));

  return BT::NodeStatus::RUNNING; // Retorna RUNNING para continuar afastando no próximo tick
}

void KeepDistanceNode::onHalted()
{
  // Freio de emergência caso a árvore aborte a ação
  auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
  oxebots_interfaces::msg::RobotCmdData cmd_data;
  cmd_data.id = getInput<uint32_t>("robot_id").value_or(0);
  cmd_data.x_velocity = 0.0;
  cmd_data.y_velocity = 0.0;
  cmd_data.angular_velocity = 0.0;
  cmd_msg->robots.push_back(cmd_data);
  cmd_pub_->publish(std::move(cmd_msg));
}
}  // namespace oxebots_strategy