#include "oxebots_strategy/goalkeeper_node.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kMaxDataAgeMs = 500.0;
constexpr double kPenaltyLineOffsetMm = 90.0;

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

namespace oxebots_strategy
{

GoalkeeperNode::GoalkeeperNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config),
  node_(node_ptr)
{
  if (!getInput<uint32_t>("robot_id", robot_id_)) {
    robot_id_ = 0;
  }
  if (!getInput<double>("kick_speed", kick_speed_)) {
    kick_speed_ = 3.0;
  }
  if (!getInput<std::string>("mode", mode_)) {
    mode_ = "normal";
  }
  if (!getInput<double>("max_hold_s", max_hold_s_)) {
    max_hold_s_ = 8.0;
  }

  if (auto blackboard = config.blackboard ? config.blackboard->rootBlackboard() : nullptr) {
    (void)blackboard->get("my_goal_x", my_goal_x_);
    (void)blackboard->get("is_yellow", is_yellow_team_);
    (void)blackboard->get("penalty_area_depth", penalty_area_depth_);
    (void)blackboard->get("penalty_area_width", penalty_area_width_);
    (void)blackboard->get("goal_width", goal_width_);
    (void)blackboard->get("field_width", field_width_);
  }

  auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>(
    "/robot_goal", goal_qos);

  auto override_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  override_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotMotionOverride>(
    "/robot_motion_override", override_qos);

  game_data_subscription_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10,
    std::bind(&GoalkeeperNode::game_data_callback, this, std::placeholders::_1));
}

BT::PortsList GoalkeeperNode::providedPorts()
{
  return {
    BT::InputPort<uint32_t>("robot_id", "{robot_id}", "ID do robô goleiro"),
    BT::InputPort<double>("kick_speed", 3.0, "Velocidade do chute em m/s"),
    BT::InputPort<std::string>(
      "mode", "normal", "Modo de operação do goleiro: normal, penalty_def ou hold"),
    BT::InputPort<double>(
      "max_hold_s", 8.0,
      "Tempo máximo (s) com a bola na área antes de forçar afastamento urgente (regra dos 10 s)")
  };
}

void GoalkeeperNode::game_data_callback(
  const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_game_data_ = msg;
  last_game_data_stamp_ = node_->now();
}

void GoalkeeperNode::clampInsidePenaltyArea(double & tx, double & ty, double margin_mm)
{
  double half_width = penalty_area_width_ / 2.0;
  if (my_goal_x_ > 0) {
    double min_x = my_goal_x_ - penalty_area_depth_ - margin_mm;
    double max_x = my_goal_x_;
    tx = std::clamp(tx, min_x, max_x);
  } else {
    double min_x = my_goal_x_;
    double max_x = my_goal_x_ + penalty_area_depth_ + margin_mm;
    tx = std::clamp(tx, min_x, max_x);
  }
  double max_y = half_width + margin_mm;
  ty = std::clamp(ty, -max_y, max_y);
}

bool GoalkeeperNode::isBallInArea(double ball_x, double ball_y, double margin_mm)
{
  double half_width = penalty_area_width_ / 2.0;
  double max_y = half_width + margin_mm;
  if (std::abs(ball_y) > max_y) {
    return false;
  }

  if (my_goal_x_ > 0) {
    double min_x = my_goal_x_ - penalty_area_depth_ - margin_mm;
    double max_x = my_goal_x_;
    return ball_x >= min_x && ball_x <= max_x;
  } else {
    double min_x = my_goal_x_;
    double max_x = my_goal_x_ + penalty_area_depth_ + margin_mm;
    return ball_x >= min_x && ball_x <= max_x;
  }
}

void GoalkeeperNode::publishGoal(
  double x, double y, double target_w, uint8_t planner_type, bool disable_arrival_latch)
{
  bool pos_changed = (std::abs(x - last_target_x_) > 5.0 || std::abs(y - last_target_y_) > 5.0);
  bool ang_changed = (std::abs(normalizeAngle(target_w - last_target_w_)) > 0.05);
  bool planner_changed =
    (planner_type != last_planner_type_) || (disable_arrival_latch != last_disable_latch_);
  // Heartbeat de 200 ms para manter o watchdog de 500 ms do SafetyGate ativo enquanto em RUNNING
  bool heartbeat_due = !last_goal_publish_time_.has_value() ||
    (node_->now() - *last_goal_publish_time_).seconds() >= 0.2;

  if (pos_changed || ang_changed || planner_changed || heartbeat_due) {
    auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
    goal_msg->robot_id = robot_id_;
    goal_msg->pose.header.stamp = node_->now();
    goal_msg->pose.header.frame_id = "map";
    goal_msg->planner_type = planner_type;
    goal_msg->disable_arrival_latch = disable_arrival_latch;
    goal_msg->pose.pose.position.x = x / 1000.0;
    goal_msg->pose.pose.position.y = y / 1000.0;
    goal_msg->pose.pose.orientation.z = std::sin(target_w * 0.5);
    goal_msg->pose.pose.orientation.w = std::cos(target_w * 0.5);

    goal_pub_->publish(std::move(goal_msg));

    last_target_x_ = x;
    last_target_y_ = y;
    last_target_w_ = target_w;
    last_planner_type_ = planner_type;
    last_disable_latch_ = disable_arrival_latch;
    last_goal_publish_time_ = node_->now();
  }
}

void GoalkeeperNode::sendKickOverride(double kick_speed)
{
  auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
  msg->robot_id = robot_id_;
  msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK;
  msg->kick_speed = std::clamp(kick_speed, 0.0, 3.0);
  override_pub_->publish(std::move(msg));
  kick_override_active_ = true;
}

void GoalkeeperNode::releaseKickOverride()
{
  if (kick_override_active_) {
    auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
    msg->robot_id = robot_id_;
    msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_NONE;
    override_pub_->publish(std::move(msg));
    kick_override_active_ = false;
  }
}

BT::NodeStatus GoalkeeperNode::onStart()
{
  last_target_x_ = -99999.0;
  last_target_y_ = -99999.0;
  last_target_w_ = -99999.0;
  last_goal_publish_time_.reset();
  ball_in_area_since_.reset();

  return onRunning();
}

BT::NodeStatus GoalkeeperNode::onRunning()
{
  auto blackboard = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
  if (!blackboard) {
    releaseKickOverride();
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "GoalkeeperNode: blackboard raiz ausente");
    return BT::NodeStatus::FAILURE;
  }

  // 1. Identidade dinâmica do goleiro via porta ou blackboard (sem robot_id == 0 fixo, SC-17)
  if (!getInput<uint32_t>("robot_id", robot_id_)) {
    if (!blackboard->get("robot_id", robot_id_)) {
      if (!blackboard->get("goalkeeper_id", robot_id_) || robot_id_ == 999u) {
        releaseKickOverride();
        RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "GoalkeeperNode: robot_id ausente");
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  // Se a blackboard tem atribuição de papel para o goleiro, valida consistência (SC-17)
  bool is_gk = false;
  const bool has_is_gk = blackboard->get("is_goalkeeper", is_gk);
  uint32_t gk_id = 999u;
  const bool has_gk_id = blackboard->get("goalkeeper_id", gk_id);

  const bool invalid_by_flag = has_is_gk && !is_gk;
  const bool invalid_by_id = has_gk_id && (gk_id != robot_id_);

  if (invalid_by_flag || invalid_by_id) {
    releaseKickOverride();
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "GoalkeeperNode: robô %u não tem papel de goleiro consistente (is_gk=%d, gk_id=%u, SC-17)",
      robot_id_, static_cast<int>(is_gk), gk_id);
    return BT::NodeStatus::FAILURE;
  }

  (void)getInput<double>("kick_speed", kick_speed_);
  mode_ = "normal";
  (void)getInput<std::string>("mode", mode_);
  max_hold_s_ = 8.0;
  (void)getInput<double>("max_hold_s", max_hold_s_);

  if (mode_ != "normal" && mode_ != "penalty_def" && mode_ != "hold") {
    releaseKickOverride();
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "GoalkeeperNode: modo inválido '%s' (esperado normal, penalty_def ou hold)",
      mode_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!(max_hold_s_ > 0.0)) {
    releaseKickOverride();
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "GoalkeeperNode: max_hold_s (%.2f) deve ser positivo", max_hold_s_);
    return BT::NodeStatus::FAILURE;
  }

  // 2. Geometria obrigatória na blackboard raiz -> FAILURE se ausente ou inválida (K01)
  if (!blackboard->get("my_goal_x", my_goal_x_) || std::abs(my_goal_x_) < 1.0) {
    if (!blackboard->get("own_goal_x", my_goal_x_) || std::abs(my_goal_x_) < 1.0) {
      releaseKickOverride();
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "GoalkeeperNode: geometria do campo (my_goal_x) ausente na blackboard -> FAILURE");
      return BT::NodeStatus::FAILURE;
    }
  }
  if (!blackboard->get("penalty_area_depth", penalty_area_depth_) ||
    !blackboard->get("penalty_area_width", penalty_area_width_) ||
    !blackboard->get("goal_width", goal_width_) ||
    !blackboard->get("field_width", field_width_) ||
    penalty_area_depth_ <= 0.0 || penalty_area_width_ <= 0.0 ||
    goal_width_ <= 0.0 || field_width_ <= 0.0)
  {
    releaseKickOverride();
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "GoalkeeperNode: dimensões do campo/área ausentes ou inválidas -> FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  // 3. Verificação de presença e frescor dos dados de bola e robô (<= 500 ms -> FAILURE se faltar)
  double pose_age_ms = 0.0;
  if (blackboard->get("pose_age_ms", pose_age_ms) && pose_age_ms > kMaxDataAgeMs) {
    releaseKickOverride();
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "GoalkeeperNode: pose_age_ms=%.1f > %.1f ms -> FAILURE", pose_age_ms, kMaxDataAgeMs);
    return BT::NodeStatus::FAILURE;
  }
  double ball_age_ms = 0.0;
  if (blackboard->get("ball_age_ms", ball_age_ms) && ball_age_ms > kMaxDataAgeMs) {
    releaseKickOverride();
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "GoalkeeperNode: ball_age_ms=%.1f > %.1f ms -> FAILURE", ball_age_ms, kMaxDataAgeMs);
    return BT::NodeStatus::FAILURE;
  }

  const rclcpp::Time now = node_->now();
  double ball_x = 0.0;
  double ball_y = 0.0;
  double rx = 0.0;
  double ry = 0.0;
  double ryaw = 0.0;
  bool have_ball = false;
  bool have_robot = false;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (last_game_data_) {
      const double gd_age_ms = (now - last_game_data_stamp_).seconds() * 1000.0;
      if (gd_age_ms > kMaxDataAgeMs) {
        releaseKickOverride();
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "GoalkeeperNode: /game_data expirado (%.1f ms > %.1f ms) -> FAILURE",
          gd_age_ms, kMaxDataAgeMs);
        return BT::NodeStatus::FAILURE;
      }
      ball_x = last_game_data_->ball.x;
      ball_y = last_game_data_->ball.y;
      have_ball = true;
      for (const auto & ally : last_game_data_->robots.allies) {
        if (ally.id == robot_id_) {
          rx = ally.x;
          ry = ally.y;
          ryaw = ally.orientation;
          have_robot = true;
          break;
        }
      }
    }
  }

  if (!have_ball) {
    have_ball = blackboard->get("ball_x", ball_x) && blackboard->get("ball_y", ball_y);
  }
  if (!have_robot) {
    have_robot = blackboard->get("robot_x", rx) &&
      blackboard->get("robot_y", ry) &&
      blackboard->get("robot_yaw", ryaw);
  }

  if (!have_ball || !have_robot) {
    releaseKickOverride();
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "GoalkeeperNode: sem dados de bola/robô %u -> FAILURE", robot_id_);
    return BT::NodeStatus::FAILURE;
  }

  constexpr double kGkMarginMm = 0.0;

  // --- MODO HOLD (K02): mantém a pose atual dentro da área e solta override ---
  if (mode_ == "hold") {
    releaseKickOverride();
    ball_in_area_since_.reset();
    double target_x = rx;
    double target_y = ry;
    clampInsidePenaltyArea(target_x, target_y, kGkMarginMm);
    publishGoal(
      target_x, target_y, ryaw,
      oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE, true);
    return BT::NodeStatus::RUNNING;
  }

  // --- MODO PENALTY_DEF (K02, SC-10): cravado na linha de gol (90 mm para dentro), nunca sai na bola ---
  if (mode_ == "penalty_def") {
    releaseKickOverride();
    ball_in_area_since_.reset();
    const double dir = (my_goal_x_ < 0.0) ? 1.0 : -1.0;
    double target_x = my_goal_x_ + dir * kPenaltyLineOffsetMm;
    const double limit_y = std::max(0.0, (goal_width_ / 2.0) - kPenaltyLineOffsetMm);
    double target_y = std::clamp(ball_y, -limit_y, limit_y);
    double target_w = (my_goal_x_ < 0.0) ? 0.0 : M_PI;
    if (std::hypot(ball_x - target_x, ball_y - target_y) > 10.0) {
      target_w = std::atan2(ball_y - target_y, ball_x - target_x);
    }
    publishGoal(
      target_x, target_y, target_w,
      oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE, true);
    return BT::NodeStatus::RUNNING;
  }

  // --- MODO NORMAL (com limite de posse de 10 s na área: max_hold_s_, K01 / SC-25) ---
  bool ball_in_area = isBallInArea(ball_x, ball_y, kGkMarginMm);

  if (ball_in_area) {
    if (!ball_in_area_since_.has_value()) {
      ball_in_area_since_ = now;
    }
    const double held_s = (now - *ball_in_area_since_).seconds();
    const bool urgent_clear = (held_s >= max_hold_s_);

    // --- ESTADO 1: Bola na área -> Sair, ir até a bola e afastar para as laterais ---
    double lateral_y = (field_width_ / 2.0) + 500.0;
    double target_side_y = (ball_y >= 0.0) ? lateral_y : -lateral_y;
    double target_side_x = 0.0;  // Meio de campo

    double clear_angle = std::atan2(target_side_y - ball_y, target_side_x - ball_x);
    double ux = std::cos(clear_angle);
    double uy = std::sin(clear_angle);

    double dx = ball_x - rx;
    double dy = ball_y - ry;
    double dist_to_ball = std::hypot(dx, dy);

    // No modo urgente (bola retida >= max_hold_s_), se o goleiro está entre o gol e a bola,
    // avança direto na direção robô->bola para limpar imediatamente antes dos 10 s regulamentares
    const double forward_sign = (my_goal_x_ < 0.0) ? 1.0 : -1.0;
    const bool behind_ball_for_urgent =
      urgent_clear && (dist_to_ball > 1.0) && (forward_sign * dx > 0.0);
    if (behind_ball_for_urgent) {
      clear_angle = std::atan2(dy, dx);
      ux = std::cos(clear_angle);
      uy = std::sin(clear_angle);
    }

    double dot = (dx * ux + dy * uy);
    double angle_err = normalizeAngle(clear_angle - ryaw);

    if (behind_ball_for_urgent || (dot > 0.0 && dist_to_ball < 250.0)) {
      // Já posicionado atrás da bola (ou em limpeza urgente atrás da bola): avança em linha reta para contato
      double target_x = ball_x + ux * 60.0;
      double target_y = ball_y + uy * 60.0;
      clampInsidePenaltyArea(target_x, target_y, kGkMarginMm);

      publishGoal(
        target_x, target_y, clear_angle,
        oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE);

      const double max_kick_dist = urgent_clear ? 220.0 : 180.0;
      const double max_angle_err = urgent_clear ? 0.85 : 0.45;
      if (dist_to_ball < max_kick_dist && std::abs(angle_err) < max_angle_err) {
        if (urgent_clear) {
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "GoalkeeperNode: afastamento urgente (bola na área há %.1fs >= %.1fs)",
            held_s, max_hold_s_);
        }
        sendKickOverride(kick_speed_);
      } else {
        releaseKickOverride();
      }
    } else {
      // Robô ainda longe ou à frente da bola (inclusive em urgent_clear quando adiantado):
      // navega até o ponto de pré-chute atrás da bola contornando-a antes de avançar em linha reta
      releaseKickOverride();
      double target_x = ball_x - ux * 200.0;
      double target_y = ball_y - uy * 200.0;
      if (dot <= 0.0 && std::abs(-dx * uy + dy * ux) < 150.0) {
        const double detour_side = (ry >= ball_y) ? 1.0 : -1.0;
        target_y = ball_y + detour_side * 180.0;
      }
      clampInsidePenaltyArea(target_x, target_y, kGkMarginMm);

      publishGoal(
        target_x, target_y, clear_angle,
        oxebots_interfaces::msg::RobotGoal::PLANNER_DSTAR);
    }
  } else {
    // --- ESTADO 2: Bola fora da área -> Defende na linha do gol acompanhando Y ---
    ball_in_area_since_.reset();
    releaseKickOverride();

    double offset_x = (my_goal_x_ > 0) ? -120.0 : 120.0;
    double target_x = my_goal_x_ + offset_x;

    double limit_y = (goal_width_ / 2.0) - 50.0;
    double target_y = std::clamp(ball_y, -limit_y, limit_y);

    double target_w = std::atan2(ball_y - target_y, ball_x - target_x);

    clampInsidePenaltyArea(target_x, target_y, kGkMarginMm);
    publishGoal(
      target_x, target_y, target_w,
      oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE);
  }

  return BT::NodeStatus::RUNNING;
}

void GoalkeeperNode::onHalted()
{
  releaseKickOverride();
  ball_in_area_since_.reset();
  RCLCPP_INFO(node_->get_logger(), "GoalkeeperNode: Robô %u parado.", robot_id_);
}

}  // namespace oxebots_strategy
