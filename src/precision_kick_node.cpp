#include "oxebots_strategy/precision_kick_node.h"
#include <algorithm>
#include <cmath>

namespace oxebots_strategy
{

PrecisionKickNode::PrecisionKickNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  override_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotMotionOverride>("/robot_motion_override", qos);
  status_sub_ = node_->create_subscription<oxebots_interfaces::msg::RobotMotionStatus>(
    "/robot_motion_status", 10, std::bind(&PrecisionKickNode::statusCallback, this, std::placeholders::_1));
}

void PrecisionKickNode::statusCallback(const oxebots_interfaces::msg::RobotMotionStatus::SharedPtr msg)
{
  if (msg->robot_id != robot_id_) return;
  last_status_ = msg;
}

BT::PortsList PrecisionKickNode::providedPorts()
{
  return { BT::InputPort<unsigned int>("robot_id", "{robot_id}", "ID do robo"),
           BT::InputPort<double>("target_x", "X do alvo"),
           BT::InputPort<double>("target_y", "Y do alvo"),
           BT::InputPort<double>("kick_speed", 3.0, "Velocidade do chute (m/s, limitada a <= 3.0)") };
}

BT::NodeStatus PrecisionKickNode::onStart()
{
  if (!getInput<unsigned int>("robot_id", robot_id_)) return BT::NodeStatus::FAILURE;

  aligned_latched_ = false;
  baseline_kick_count_ = (last_status_ && last_status_->robot_id == robot_id_) ? last_status_->kick_fire_count : 0;

  return step();
}

BT::NodeStatus PrecisionKickNode::onRunning()
{
  return step();
}

BT::NodeStatus PrecisionKickNode::step()
{
  if (!getInput<unsigned int>("robot_id", robot_id_)) return BT::NodeStatus::FAILURE;

  double target_x, target_y, kick_speed = 3.0;
  if (!getInput<double>("target_x", target_x)) return BT::NodeStatus::FAILURE;
  if (!getInput<double>("target_y", target_y)) return BT::NodeStatus::FAILURE;
  getInput<double>("kick_speed", kick_speed);

  // Regra R-25 / SC-24 (M02): nunca exceder 3,0 m/s
  kick_speed = std::clamp(kick_speed, 0.0, 3.0);

  // Se o controlador já concluiu o disparo do chute desde onStart(), libera o override e encerra com SUCCESS
  if (last_status_ &&
      last_status_->robot_id == robot_id_ &&
      last_status_->kick_fire_count > baseline_kick_count_) {
    releaseOverride();
    RCLCPP_INFO(node_->get_logger(), "PrecisionKick (Robot %d): chute concluído via override.", robot_id_);
    return BT::NodeStatus::SUCCESS;
  }

  auto bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
  if (!bb) return BT::NodeStatus::FAILURE;
  double ryaw = 0.0, bx = 0.0, by = 0.0;
  if (!bb->get("robot_yaw", ryaw)) return BT::NodeStatus::FAILURE;
  if (!bb->get("ball_x", bx)) return BT::NodeStatus::FAILURE;
  if (!bb->get("ball_y", by)) return BT::NodeStatus::FAILURE;

  // 1. Vetor Bola -> Alvo (direção do chute ideal)
  double dx_kick = target_x - bx;
  double dy_kick = target_y - by;
  double angle_kick = std::atan2(dy_kick, dx_kick);
  double angle_error = normalize_angle(angle_kick - ryaw);

  // Verifica se o controlador reportou alinhamento concluído ou se já está dentro da tolerância (~8.5 graus)
  if (!aligned_latched_) {
    bool status_aligned = last_status_ &&
                          last_status_->robot_id == robot_id_ &&
                          last_status_->active_mode == oxebots_interfaces::msg::RobotMotionOverride::MODE_ALIGN_TO_ANGLE &&
                          last_status_->aligned;
    if (status_aligned || std::abs(angle_error) < 0.15) {
      aligned_latched_ = true;
    }
  }

  auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
  msg->robot_id = robot_id_;

  if (!aligned_latched_) {
    // Fase 1: Pede ao controlador para girar até o ângulo bola->alvo
    msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_ALIGN_TO_ANGLE;
    msg->target_angle = angle_kick;
    msg->angle_threshold_deg = 8.0;
    msg->max_angular_speed = 3.0;
    msg->p_gain = 4.0;
  } else {
    // Fase 2: Alinhado -> pede o disparo do chute ao controlador
    msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK;
    msg->kick_speed = kick_speed;
  }

  override_pub_->publish(std::move(msg));
  return BT::NodeStatus::RUNNING;
}

void PrecisionKickNode::releaseOverride()
{
  auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
  msg->robot_id = robot_id_;
  msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_NONE;
  override_pub_->publish(std::move(msg));
}

void PrecisionKickNode::onHalted()
{
  releaseOverride();
}

double PrecisionKickNode::normalize_angle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

} // namespace oxebots_strategy
