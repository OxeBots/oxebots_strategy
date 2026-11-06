#include "oxebots_strategy/avoid_placement_corridor_node.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace oxebots_strategy
{

namespace
{

constexpr double kEscapeExtraMm = 50.0;
constexpr double kMinimumMarginMm = 500.0;
constexpr double kMinimumSegmentLengthMm = 1e-6;

struct Point
{
  double x;
  double y;
};

double squaredDistance(const Point & lhs, const Point & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return dx * dx + dy * dy;
}

bool corridorEscapeTarget(
  const Point & robot,
  const Point & ball,
  const Point & designated,
  double margin_mm,
  Point & target)
{
  const Point segment{
    designated.x - ball.x,
    designated.y - ball.y};
  const double segment_length_sq = segment.x * segment.x + segment.y * segment.y;

  Point closest = ball;
  double segment_parameter = 0.0;
  if (segment_length_sq > kMinimumSegmentLengthMm * kMinimumSegmentLengthMm) {
    segment_parameter =
      ((robot.x - ball.x) * segment.x + (robot.y - ball.y) * segment.y) / segment_length_sq;
    segment_parameter = std::clamp(segment_parameter, 0.0, 1.0);
    closest = {
      ball.x + segment_parameter * segment.x,
      ball.y + segment_parameter * segment.y};
  }

  const double distance_sq = squaredDistance(robot, closest);
  const double required_distance = margin_mm + kEscapeExtraMm;
  if (distance_sq >= margin_mm * margin_mm) {
    return false;
  }

  Point direction{
    robot.x - closest.x,
    robot.y - closest.y};
  double direction_length = std::sqrt(distance_sq);

  if (direction_length < kMinimumSegmentLengthMm) {
    if (segment_length_sq > kMinimumSegmentLengthMm * kMinimumSegmentLengthMm) {
      const double segment_length = std::sqrt(segment_length_sq);
      direction = {-segment.y / segment_length, segment.x / segment_length};
    } else {
      direction = {1.0, 0.0};
    }
    direction_length = 1.0;
  }

  target = {
    closest.x + direction.x / direction_length * required_distance,
    closest.y + direction.y / direction_length * required_distance};
  return true;
}

}  // namespace

AvoidPlacementCorridorNode::AvoidPlacementCorridorNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(std::move(node))
{
  const auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>(
    "/robot_goal", goal_qos);
}

BT::PortsList AvoidPlacementCorridorNode::providedPorts()
{
  return {
    BT::InputPort<uint32_t>("robot_id", "{robot_id}", "Id do robô"),
    BT::InputPort<double>("robot_x", "{robot_x}", "Posição X atual do robô (mm)"),
    BT::InputPort<double>("robot_y", "{robot_y}", "Posição Y atual do robô (mm)"),
    BT::InputPort<double>("robot_yaw", "{robot_yaw}", "Orientação atual do robô (rad)"),
    BT::InputPort<double>("ball_x", "{ball_x}", "Posição X da bola (mm)"),
    BT::InputPort<double>("ball_y", "{ball_y}", "Posição Y da bola (mm)"),
    BT::InputPort<double>(
      "designated_x", "{designated_x}", "Posição X designada para a bola (mm)"),
    BT::InputPort<double>(
      "designated_y", "{designated_y}", "Posição Y designada para a bola (mm)"),
    BT::InputPort<bool>(
      "has_designated", "{has_designated}", "Indica se existe posição designada válida"),
    BT::InputPort<double>(
      "margin_mm", 550.0, "Margem do corredor proibido (mm); deve ser >= 500"),
    BT::InputPort<double>(
      "max_speed", 0.75, "Teto de velocidade do deslocamento (m/s)")
  };
}

bool AvoidPlacementCorridorNode::readInputs(
  uint32_t & robot_id,
  double & robot_x,
  double & robot_y,
  double & robot_yaw,
  double & ball_x,
  double & ball_y,
  double & designated_x,
  double & designated_y,
  bool & has_designated,
  double & margin_mm,
  double & max_speed)
{
  const bool valid =
    getInput<uint32_t>("robot_id", robot_id) &&
    getInput<double>("robot_x", robot_x) &&
    getInput<double>("robot_y", robot_y) &&
    getInput<double>("robot_yaw", robot_yaw) &&
    getInput<double>("ball_x", ball_x) &&
    getInput<double>("ball_y", ball_y) &&
    getInput<double>("designated_x", designated_x) &&
    getInput<double>("designated_y", designated_y) &&
    getInput<bool>("has_designated", has_designated) &&
    getInput<double>("margin_mm", margin_mm) &&
    getInput<double>("max_speed", max_speed);

  if (!valid) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "AvoidPlacementCorridor: dados obrigatorios ausentes na blackboard");
    return false;
  }

  if (!std::isfinite(robot_x) || !std::isfinite(robot_y) ||
      !std::isfinite(robot_yaw) || !std::isfinite(ball_x) ||
      !std::isfinite(ball_y) || !std::isfinite(designated_x) ||
      !std::isfinite(designated_y) || !std::isfinite(margin_mm) ||
      !std::isfinite(max_speed)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "AvoidPlacementCorridor: dados nao finitos na blackboard");
    return false;
  }

  if (margin_mm < kMinimumMarginMm || max_speed <= 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "AvoidPlacementCorridor: margin_mm deve ser >= %.1f e max_speed > 0",
      kMinimumMarginMm);
    return false;
  }

  if (!has_designated) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "AvoidPlacementCorridor: posição designada ausente");
    return false;
  }

  return true;
}

bool AvoidPlacementCorridorNode::publishGoal(
  uint32_t robot_id,
  double target_x,
  double target_y,
  double target_yaw,
  double max_speed)
{
  if (!std::isfinite(target_x) || !std::isfinite(target_y) ||
      !std::isfinite(target_yaw)) {
    return false;
  }

  oxebots_interfaces::msg::RobotGoal goal;
  goal.robot_id = static_cast<int32_t>(robot_id);
  goal.pose.header.stamp = node_->now();
  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = target_x / 1000.0;
  goal.pose.pose.position.y = target_y / 1000.0;
  goal.pose.pose.orientation.z = std::sin(target_yaw * 0.5);
  goal.pose.pose.orientation.w = std::cos(target_yaw * 0.5);
  goal.planner_type = oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE;
  goal.disable_arrival_latch = false;
  goal.max_speed = static_cast<float>(max_speed);
  goal_pub_->publish(goal);
  return true;
}

BT::NodeStatus AvoidPlacementCorridorNode::update()
{
  uint32_t robot_id = 0;
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  double ball_x = 0.0;
  double ball_y = 0.0;
  double designated_x = 0.0;
  double designated_y = 0.0;
  bool has_designated = false;
  double margin_mm = 0.0;
  double max_speed = 0.0;

  if (!readInputs(
      robot_id, robot_x, robot_y, robot_yaw, ball_x, ball_y,
      designated_x, designated_y, has_designated, margin_mm, max_speed)) {
    return BT::NodeStatus::FAILURE;
  }

  Point target{};
  const bool needs_escape = corridorEscapeTarget(
    {robot_x, robot_y}, {ball_x, ball_y}, {designated_x, designated_y},
    margin_mm, target);
  if (!needs_escape) {
    return BT::NodeStatus::SUCCESS;
  }

  if (!publishGoal(robot_id, target.x, target.y, robot_yaw, max_speed)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "AvoidPlacementCorridor: nao foi possivel publicar alvo de afastamento");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AvoidPlacementCorridorNode::onStart()
{
  return update();
}

BT::NodeStatus AvoidPlacementCorridorNode::onRunning()
{
  return update();
}

void AvoidPlacementCorridorNode::onHalted()
{
  // A folha nao usa override; a proxima arvore deve publicar o objetivo vigente.
}

}  // namespace oxebots_strategy
