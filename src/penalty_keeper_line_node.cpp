#include "oxebots_strategy/penalty_keeper_line_node.hpp"

#include <algorithm>
#include <cmath>

namespace oxebots_strategy
{

namespace
{
constexpr double kRobotRadiusMm = 90.0;
constexpr double kPosThresholdMm = 5.0;
constexpr double kAngThresholdRad = 0.05;
constexpr double kHeartbeatS = 0.2;
constexpr double kMaxDataAgeMs = 500.0;

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}
}  // namespace

PenaltyKeeperLineNode::PenaltyKeeperLineNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node)
{
  if (node_) {
    if (!node_->has_parameter("line_offset_mm")) {
      default_line_offset_mm_ = node_->declare_parameter<double>("line_offset_mm", 90.0);
    } else {
      default_line_offset_mm_ = node_->get_parameter("line_offset_mm").as_double();
    }

    auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>(
      "/robot_goal", goal_qos);
    game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
      "/game_data", 10,
      std::bind(&PenaltyKeeperLineNode::gameDataCallback, this, std::placeholders::_1));
  }
}

BT::PortsList PenaltyKeeperLineNode::providedPorts()
{
  return {
    BT::InputPort<uint32_t>(
      "robot_id", "{robot_id}", "Id do robô goleiro (padrão {robot_id} da blackboard)"),
    BT::InputPort<std::string>(
      "side", "defend", "Lado de atuação no pênalti (defend)"),
    BT::InputPort<double>(
      "line_offset_mm", 90.0,
      "Distância (mm) da linha de gol para dentro do campo (padrão 90 mm = 1 raio do robô)")
  };
}

void PenaltyKeeperLineNode::gameDataCallback(
  const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_game_data_ = msg;
  if (node_) {
    last_game_data_stamp_ = node_->now();
  }
}

BT::NodeStatus PenaltyKeeperLineNode::onStart()
{
  last_target_x_ = -99999.0;
  last_target_y_ = -99999.0;
  last_target_w_ = -99999.0;
  last_publish_time_.reset();
  return step();
}

BT::NodeStatus PenaltyKeeperLineNode::onRunning()
{
  return step();
}

BT::NodeStatus PenaltyKeeperLineNode::step()
{
  if (!node_) {
    return BT::NodeStatus::FAILURE;
  }

  auto * bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
  if (!bb) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "PenaltyKeeperLine: blackboard raiz ausente");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput<uint32_t>("robot_id", robot_id_)) {
    if (!bb->get("robot_id", robot_id_)) {
      if (!bb->get("goalkeeper_id", robot_id_) || robot_id_ == 999u) {
        RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "PenaltyKeeperLine: robot_id ausente");
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  // Se a blackboard já atribuiu papéis, garante consistência de que este robô é o goleiro (SC-17)
  bool is_gk = false;
  const bool has_is_gk = bb->get("is_goalkeeper", is_gk);
  uint32_t gk_id = 999u;
  const bool has_gk_id = bb->get("goalkeeper_id", gk_id);

  const bool invalid_by_flag = has_is_gk && !is_gk;
  const bool invalid_by_id = has_gk_id && (gk_id != robot_id_);

  if (invalid_by_flag || invalid_by_id) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "PenaltyKeeperLine: robô %u não possui papel de goleiro consistente (is_gk=%d, gk_id=%u, SC-17)",
      robot_id_, static_cast<int>(is_gk), gk_id);
    return BT::NodeStatus::FAILURE;
  }

  std::string side = "defend";
  (void)getInput<std::string>("side", side);
  if (side != "defend") {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "PenaltyKeeperLine: side inválido '%s' (esperado 'defend')", side.c_str());
    return BT::NodeStatus::FAILURE;
  }

  double line_offset_mm = default_line_offset_mm_;
  if (node_->has_parameter("line_offset_mm")) {
    line_offset_mm = node_->get_parameter("line_offset_mm").as_double();
  }
  if (config().input_ports.find("line_offset_mm") != config().input_ports.end()) {
    (void)getInput<double>("line_offset_mm", line_offset_mm);
  }
  if (line_offset_mm < 0.0) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "PenaltyKeeperLine: line_offset_mm (%.1f) não pode ser negativo", line_offset_mm);
    return BT::NodeStatus::FAILURE;
  }

  double my_goal_x = 0.0;
  if (!bb->get("my_goal_x", my_goal_x) || std::abs(my_goal_x) < 1.0) {
    if (!bb->get("own_goal_x", my_goal_x) || std::abs(my_goal_x) < 1.0) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "PenaltyKeeperLine: aguardando geometria do campo (my_goal_x/own_goal_x)");
      return BT::NodeStatus::FAILURE;
    }
  }

  double goal_width = 0.0;
  if (!bb->get("goal_width", goal_width) || goal_width <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "PenaltyKeeperLine: goal_width ausente ou inválido");
    return BT::NodeStatus::FAILURE;
  }

  double pose_age_ms = 0.0;
  if (bb->get("pose_age_ms", pose_age_ms) && pose_age_ms > kMaxDataAgeMs) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "PenaltyKeeperLine: pose_age_ms=%.1f > %.1f ms", pose_age_ms, kMaxDataAgeMs);
    return BT::NodeStatus::FAILURE;
  }
  double ball_age_ms = 0.0;
  if (bb->get("ball_age_ms", ball_age_ms) && ball_age_ms > kMaxDataAgeMs) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "PenaltyKeeperLine: ball_age_ms=%.1f > %.1f ms", ball_age_ms, kMaxDataAgeMs);
    return BT::NodeStatus::FAILURE;
  }

  double ball_x = 0.0;
  double ball_y = 0.0;
  bool have_ball = bb->get("ball_x", ball_x) && bb->get("ball_y", ball_y);

  oxebots_interfaces::msg::GameData::SharedPtr gd;
  rclcpp::Time gd_stamp{0, 0, RCL_ROS_TIME};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gd = last_game_data_;
    gd_stamp = last_game_data_stamp_;
  }

  const rclcpp::Time now = node_->now();
  if (gd) {
    const double gd_age_ms = (now - gd_stamp).seconds() * 1000.0;
    if (gd_age_ms > kMaxDataAgeMs) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "PenaltyKeeperLine: /game_data expirado (%.1f ms)", gd_age_ms);
      return BT::NodeStatus::FAILURE;
    }
    ball_x = gd->ball.x;
    ball_y = gd->ball.y;
    have_ball = true;
  }

  if (!have_ball) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "PenaltyKeeperLine: dados da bola ausentes");
    return BT::NodeStatus::FAILURE;
  }

  // Direção para dentro do campo a partir da linha de gol (+1 se o gol está em -X, -1 se em +X)
  const double dir = (my_goal_x < 0.0) ? 1.0 : -1.0;
  const double target_x = my_goal_x + dir * line_offset_mm;

  const double limit_y = std::max(0.0, (goal_width / 2.0) - kRobotRadiusMm);
  const double target_y = std::clamp(ball_y, -limit_y, limit_y);

  double target_w = (my_goal_x < 0.0) ? 0.0 : M_PI;
  if (std::hypot(ball_x - target_x, ball_y - target_y) > 10.0) {
    target_w = std::atan2(ball_y - target_y, ball_x - target_x);
  }

  const bool pos_changed =
    std::hypot(target_x - last_target_x_, target_y - last_target_y_) > kPosThresholdMm;
  const bool ang_changed =
    std::abs(normalizeAngle(target_w - last_target_w_)) > kAngThresholdRad;
  const bool heartbeat_due = !last_publish_time_.has_value() ||
    (now - *last_publish_time_).seconds() >= kHeartbeatS;

  if (pos_changed || ang_changed || heartbeat_due) {
    auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
    goal_msg->robot_id = robot_id_;
    goal_msg->pose.header.stamp = now;
    goal_msg->pose.header.frame_id = "map";
    goal_msg->planner_type = oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE;
    goal_msg->disable_arrival_latch = true;
    goal_msg->pose.pose.position.x = target_x / 1000.0;
    goal_msg->pose.pose.position.y = target_y / 1000.0;
    goal_msg->pose.pose.orientation.z = std::sin(target_w * 0.5);
    goal_msg->pose.pose.orientation.w = std::cos(target_w * 0.5);
    goal_pub_->publish(std::move(goal_msg));

    last_target_x_ = target_x;
    last_target_y_ = target_y;
    last_target_w_ = target_w;
    last_publish_time_ = now;
  }

  return BT::NodeStatus::RUNNING;
}

void PenaltyKeeperLineNode::onHalted()
{
  last_target_x_ = -99999.0;
  last_target_y_ = -99999.0;
  last_target_w_ = -99999.0;
  last_publish_time_.reset();
}

}  // namespace oxebots_strategy
