#include "oxebots_strategy/kick_once_node.hpp"

#include <algorithm>
#include <cmath>

namespace oxebots_strategy
{

namespace
{
double normalizeAngleDeg(double angle)
{
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}
}  // namespace

KickOnceNode::KickOnceNode(const std::string & name, const BT::NodeConfig & config,
                           rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node), start_time_(0, 0, RCL_ROS_TIME)
{
  override_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotMotionOverride>(
    "/robot_motion_override", rclcpp::QoS(rclcpp::KeepLast(1)));
}

BT::PortsList KickOnceNode::providedPorts()
{
  return {
    BT::InputPort<unsigned int>("robot_id", "{robot_id}", "Id do robô que chuta"),
    BT::InputPort<double>("target_x", "Opcional: X do alvo do chute (mm); só para o log de alinhamento"),
    BT::InputPort<double>("target_y", "Opcional: Y do alvo do chute (mm); só para o log de alinhamento"),
    BT::InputPort<double>("kick_speed", kDefaultKickSpeed, "Velocidade do chute (m/s); limitada a 3,0"),
    BT::InputPort<double>("timeout_s", 10.0, "Prazo (s) para a bola se mover; passado, FAILURE"),
    BT::InputPort<double>("ball_moved_mm", 50.0, "Deslocamento da bola (mm) que confirma o chute")};
}

bool KickOnceNode::readBall(double & x, double & y) const
{
  auto * bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
  return bb && bb->get("ball_x", x) && bb->get("ball_y", y);
}

BT::NodeStatus KickOnceNode::onStart()
{
  if (!getInput("robot_id", robot_id_)) {
    RCLCPP_ERROR(node_->get_logger(), "KickOnce: porta [robot_id] ausente ou inválida.");
    return BT::NodeStatus::FAILURE;
  }

  // Um {kick_speed} sem valor na blackboard (subárvore chamada sem a porta) cai no padrão.
  double kick_speed = kDefaultKickSpeed;
  if (!getInput("kick_speed", kick_speed)) {
    kick_speed = kDefaultKickSpeed;
  }
  if (!std::isfinite(kick_speed) || kick_speed <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "KickOnce (robô %u): kick_speed inválido (%.2f).",
                 robot_id_, kick_speed);
    return BT::NodeStatus::FAILURE;
  }
  if (kick_speed > kMaxKickSpeed) {
    RCLCPP_WARN(node_->get_logger(),
                "KickOnce (robô %u): kick_speed %.2f m/s acima do teto; limitado a %.1f m/s.",
                robot_id_, kick_speed, kMaxKickSpeed);
    kick_speed = kMaxKickSpeed;
  }
  kick_speed_ = kick_speed;

  timeout_s_ = 10.0;
  ball_moved_mm_ = 50.0;
  (void)getInput("timeout_s", timeout_s_);
  (void)getInput("ball_moved_mm", ball_moved_mm_);
  if (!(timeout_s_ > 0.0) || !(ball_moved_mm_ > 0.0)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "KickOnce (robô %u): timeout_s (%.2f) e ball_moved_mm (%.1f) devem ser positivos.",
                 robot_id_, timeout_s_, ball_moved_mm_);
    return BT::NodeStatus::FAILURE;
  }

  // A referência é a bola NO MOMENTO do pedido: só o deslocamento causado por este chute conta.
  if (!readBall(ball_start_x_, ball_start_y_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "KickOnce (robô %u): ball_x/ball_y ausentes na blackboard; sem a bola não há como "
                 "confirmar o chute.", robot_id_);
    return BT::NodeStatus::FAILURE;
  }
  start_time_ = node_->now();

  // Diagnóstico: erro entre a direção do robô e a do alvo. Chute torto é a causa mais comum de
  // "chutou e a bola não foi para onde devia".
  double tx = 0.0, ty = 0.0, rx = 0.0, ry = 0.0, ryaw = 0.0;
  auto * bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
  if (getInput("target_x", tx) && getInput("target_y", ty) && bb &&
      bb->get("robot_x", rx) && bb->get("robot_y", ry) && bb->get("robot_yaw", ryaw)) {
    const double err_deg = normalizeAngleDeg(std::atan2(ty - ry, tx - rx) * 180.0 / M_PI -
                                             ryaw * 180.0 / M_PI);
    RCLCPP_INFO(node_->get_logger(),
                "KickOnce (robô %u): vel=%.2f m/s, alvo=(%.0f,%.0f), erro de alinhamento=%.1f graus.",
                robot_id_, kick_speed_, tx, ty, err_deg);
  } else {
    RCLCPP_INFO(node_->get_logger(), "KickOnce (robô %u): pedindo chute (vel=%.2f m/s).",
                robot_id_, kick_speed_);
  }

  return step();
}

BT::NodeStatus KickOnceNode::onRunning()
{
  return step();
}

BT::NodeStatus KickOnceNode::step()
{
  double bx = 0.0;
  double by = 0.0;
  if (!readBall(bx, by)) {
    releaseOverride();
    RCLCPP_ERROR(node_->get_logger(), "KickOnce (robô %u): perdi ball_x/ball_y na blackboard.",
                 robot_id_);
    return BT::NodeStatus::FAILURE;
  }

  const double moved = std::hypot(bx - ball_start_x_, by - ball_start_y_);
  if (moved >= ball_moved_mm_) {
    // Libera o override explicitamente: o Repeat/Retry da árvore pode reiniciar a sequência sem
    // chamar onHalted(), e o MODE_KICK ficaria grudado até o TTL do controlador.
    releaseOverride();
    RCLCPP_INFO(node_->get_logger(), "KickOnce (robô %u): bola se moveu %.0f mm, chute confirmado.",
                robot_id_, moved);
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed = (node_->now() - start_time_).seconds();
  if (elapsed >= timeout_s_) {
    releaseOverride();
    RCLCPP_WARN(node_->get_logger(),
                "KickOnce (robô %u): bola não se moveu em %.1f s (deslocamento %.0f mm); FAILURE.",
                robot_id_, timeout_s_, moved);
    return BT::NodeStatus::FAILURE;
  }

  requestKick();
  return BT::NodeStatus::RUNNING;
}

void KickOnceNode::requestKick()
{
  auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
  msg->robot_id = robot_id_;
  msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK;
  msg->kick_speed = kick_speed_;
  override_pub_->publish(std::move(msg));
}

void KickOnceNode::releaseOverride()
{
  auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
  msg->robot_id = robot_id_;
  msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_NONE;
  override_pub_->publish(std::move(msg));
}

void KickOnceNode::onHalted()
{
  releaseOverride();
  RCLCPP_WARN(node_->get_logger(), "KickOnce (robô %u) interrompido; chute cancelado.", robot_id_);
}

}  // namespace oxebots_strategy
