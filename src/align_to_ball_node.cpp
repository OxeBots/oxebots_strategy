#include "oxebots_strategy/align_to_ball_node.h"

namespace oxebots_strategy
{

AlignToBallNode::AlignToBallNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  override_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotMotionOverride>("/robot_motion_override", qos);
  status_sub_ = node_->create_subscription<oxebots_interfaces::msg::RobotMotionStatus>(
    "/robot_motion_status", 10, std::bind(&AlignToBallNode::statusCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "AlignToBallNode configurado.");
}

void AlignToBallNode::statusCallback(const oxebots_interfaces::msg::RobotMotionStatus::SharedPtr msg)
{
  // Filtra aqui (não na leitura) para nunca guardar por engano o status de outro robô: o
  // tópico é compartilhado entre todas as instâncias do controlador, uma por robô.
  if (msg->robot_id != robot_id_) return;
  last_status_ = msg;
}

BT::PortsList AlignToBallNode::providedPorts()
{
  return { BT::InputPort<unsigned int>("robot_id"),
           BT::InputPort<double>("angle_threshold_deg", 10.0, "Desvio angular máximo para considerar alinhado"),
           BT::InputPort<double>("max_angular_speed", 3.0, "Velocidade angular máxima (rad/s)"),
           BT::InputPort<double>("p_gain", 4.0, "Ganho proporcional do controle de giro") };
}

BT::NodeStatus AlignToBallNode::onStart()
{
  if (!getInput<unsigned int>("robot_id", robot_id_)) return BT::NodeStatus::FAILURE;
  return step();
}

BT::NodeStatus AlignToBallNode::onRunning()
{
  return step();
}

BT::NodeStatus AlignToBallNode::step()
{
  double angle_threshold_deg = 10.0, max_angular_speed = 3.0, p_gain = 4.0;
  getInput<double>("angle_threshold_deg", angle_threshold_deg);
  getInput<double>("max_angular_speed", max_angular_speed);
  getInput<double>("p_gain", p_gain);

  // Chegou alinhado desde o último pedido: libera o override em vez de renová-lo, para o
  // controlador voltar a reagir imediatamente ao próximo GoToPoint (sem isso, o modo ALIGN
  // ficaria "grudado" por até o TTL de segurança do controlador, atrasando a próxima corrida).
  if (last_status_ &&
      last_status_->robot_id == robot_id_ &&
      last_status_->active_mode == oxebots_interfaces::msg::RobotMotionOverride::MODE_ALIGN_TO_BALL &&
      last_status_->aligned) {
    releaseOverride();
    return BT::NodeStatus::SUCCESS;
  }

  auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
  msg->robot_id = robot_id_;
  msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_ALIGN_TO_BALL;
  msg->angle_threshold_deg = angle_threshold_deg;
  msg->max_angular_speed = max_angular_speed;
  msg->p_gain = p_gain;
  override_pub_->publish(std::move(msg));

  return BT::NodeStatus::RUNNING;
}

void AlignToBallNode::releaseOverride()
{
  auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
  msg->robot_id = robot_id_;
  msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_NONE;
  override_pub_->publish(std::move(msg));
}

void AlignToBallNode::onHalted()
{
  releaseOverride();
}

} // namespace oxebots_strategy
