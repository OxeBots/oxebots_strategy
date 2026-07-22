#include "oxebots_strategy/kick_ball_node.h"
#include <cmath>

namespace oxebots_strategy
{

namespace {
double normalizeAngleDeg(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}
}

KickBallNode::KickBallNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  override_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotMotionOverride>("/robot_motion_override", qos);
  status_sub_ = node_->create_subscription<oxebots_interfaces::msg::RobotMotionStatus>(
    "/robot_motion_status", 10, std::bind(&KickBallNode::statusCallback, this, std::placeholders::_1));
  game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10, std::bind(&KickBallNode::gameDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "KickBallNode configurado.");
}

void KickBallNode::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  last_game_data_ = msg;
}

void KickBallNode::statusCallback(const oxebots_interfaces::msg::RobotMotionStatus::SharedPtr msg)
{
  // Filtra aqui (não na leitura) para nunca guardar por engano o status de outro robô: o
  // tópico é compartilhado entre todas as instâncias do controlador, uma por robô.
  if (msg->robot_id != robot_id_) return;
  last_status_ = msg;
}

void KickBallNode::logKickGeometry(unsigned int robot_id)
{
  if (!last_game_data_) return;

  for (const auto& ally : last_game_data_->robots.allies) {
    if (ally.id != robot_id) continue;

    double bx = last_game_data_->ball.x;
    double by = last_game_data_->ball.y;
    double dx = bx - ally.x;
    double dy = by - ally.y;
    double dist = std::hypot(dx, dy);
    double angle_to_ball_deg = std::atan2(dy, dx) * 180.0 / M_PI;
    double robot_heading_deg = ally.orientation * 180.0 / M_PI;
    double offset_deg = normalizeAngleDeg(angle_to_ball_deg - robot_heading_deg);

    // Diagnóstico: se offset_deg for grande (ex: >15-20 graus), a bola não está no cone do
    // kicker mesmo com a distância ok — o chute sai "no vácuo" apesar do IsBallClose passar.
    RCLCPP_INFO(node_->get_logger(),
      "Robot %d GEOMETRIA DO CHUTE: robot(%.1f,%.1f) heading=%.1fdeg ball(%.1f,%.1f) dist=%.1fmm offset_angulo=%.1fdeg",
      robot_id, (double)ally.x, (double)ally.y, robot_heading_deg, bx, by, dist, offset_deg);
    return;
  }
}

BT::PortsList KickBallNode::providedPorts()
{
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

  robot_id_ = robot_id;
  kick_speed_ = kick_speed;
  baseline_kick_count_ = (last_status_ && last_status_->robot_id == robot_id_) ? last_status_->kick_fire_count : 0;
  logKickGeometry(robot_id_);
  RCLCPP_INFO(node_->get_logger(), "Robot %d: pedindo chute (vel: %.1f)", robot_id_, kick_speed_);

  return step();
}

BT::NodeStatus KickBallNode::onRunning()
{
  return step();
}

BT::NodeStatus KickBallNode::step()
{
  if (last_status_ &&
      last_status_->robot_id == robot_id_ &&
      last_status_->kick_fire_count > baseline_kick_count_) {
    // Chute concluído desde o pedido: libera o override em vez de renová-lo, para o
    // controlador voltar a reagir imediatamente ao próximo GoToPoint (o Repeat da árvore
    // reinicia a sequência sem chamar onHalted() neste nó, então essa liberação explícita é o
    // único jeito de não deixar o modo KICK grudado até o TTL de segurança do controlador).
    releaseOverride();
    RCLCPP_INFO(node_->get_logger(), "Robot %d: chute concluído.", robot_id_);
    return BT::NodeStatus::SUCCESS;
  }

  auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
  msg->robot_id = robot_id_;
  msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK;
  msg->kick_speed = kick_speed_;
  override_pub_->publish(std::move(msg));

  return BT::NodeStatus::RUNNING;
}

void KickBallNode::releaseOverride()
{
  auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
  msg->robot_id = robot_id_;
  msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_NONE;
  override_pub_->publish(std::move(msg));
}

void KickBallNode::onHalted()
{
  releaseOverride();
  RCLCPP_WARN(node_->get_logger(), "KickBallNode (Robot %d) interrompido, chute cancelado.", robot_id_);
}

} // namespace oxebots_strategy
