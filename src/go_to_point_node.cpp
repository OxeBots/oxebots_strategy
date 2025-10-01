#include "oxebots_strategy/go_to_point_node.h"

#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/robot_cmd_data.hpp"


namespace oxebots_strategy
{

// Construtor: Cria o publisher quando o nó é instanciado
GoToPointNode::GoToPointNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  publisher_ = node_->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", 10);
  RCLCPP_INFO(node_->get_logger(), "!!! Construtor do GoToPointNode executado. Publisher criado. !!!");
}

// providedPorts: Define as entradas que o nó aceita no XML da árvore de comportamento
BT::PortsList GoToPointNode::providedPorts()
{
  return { BT::InputPort<unsigned int>("robot_id"),
           BT::InputPort<double>("x"),
           BT::InputPort<double>("y"),
           BT::InputPort<double>("w") };
}

// onStart: Chamado apenas na primeira vez que o nó é executado
BT::NodeStatus GoToPointNode::onStart()
{
  // Agora, este método apenas registra que a ação começou.
  RCLCPP_INFO(node_->get_logger(), "Iniciando 'AndarParaFrente'");
  return BT::NodeStatus::RUNNING;
}

// onRunning: Chamado repetidamente enquanto a ação estiver ativa
BT::NodeStatus GoToPointNode::onRunning()
{
  // A lógica de criar e publicar a mensagem foi movida para cá.
  // Isso garante que o comando seja enviado continuamente.
  unsigned int robot_id = 0;
  getInput<unsigned int>("robot_id", robot_id);

  auto msg = oxebots_interfaces::msg::RobotCmd();
  auto robot_data = oxebots_interfaces::msg::RobotCmdData();
  
  robot_data.id = robot_id;
  
  float forward_speed = -5000000000.0;
  robot_data.x_velocity = forward_speed;  // Velocidade para frente
  robot_data.y_velocity = 0.0;            // Sem movimento lateral
  robot_data.angular_velocity = 0.0;      // Sem rotação

  msg.robots.push_back(robot_data);
  publisher_->publish(msg);
  
  // Continua retornando RUNNING para que a árvore continue executando este nó
  return BT::NodeStatus::RUNNING;
}

// onHalted: Chamado se a ação for interrompida
void GoToPointNode::onHalted()
{
  // Medida de segurança: parar o robô se a ação for cancelada
  RCLCPP_WARN(node_->get_logger(), "Ação 'AndarParaFrente' interrompida. Parando o robô.");
  
  auto msg = oxebots_interfaces::msg::RobotCmd();
  auto robot_data = oxebots_interfaces::msg::RobotCmdData();

  robot_data.id = 0; // Você pode querer usar o robot_id obtido anteriormente
  robot_data.x_velocity = 0.0;
  robot_data.y_velocity = 0.0;
  robot_data.angular_velocity = 0.0;
  
  msg.robots.push_back(robot_data);
  publisher_->publish(msg);
}

}  // namespace oxebots_strategy