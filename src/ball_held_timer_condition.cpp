#include "oxebots_strategy/ball_held_timer_condition.hpp"

#include <cmath>
#include "rclcpp/rclcpp.hpp"

namespace oxebots_strategy
{

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("ball_held_timer");

bool isInsidePenaltyArea(
  double bx, double by, double goal_x, double area_depth, double area_width)
{
  if (std::abs(by) > area_width / 2.0) {
    return false;
  }
  if (goal_x > 0.0) {
    return bx >= (goal_x - area_depth) && bx <= goal_x;
  }
  return bx >= goal_x && bx <= (goal_x + area_depth);
}
}  // namespace

BallHeldTimerCondition::BallHeldTimerCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList BallHeldTimerCondition::providedPorts()
{
  return {
    BT::InputPort<double>(
      "max_s", 8.0, "Tempo máximo (s) que a bola pode ficar parada na área antes de falhar"),
    BT::InputPort<std::string>(
      "area", "own_defense", "Área monitorada: own_defense ou opp_defense")
  };
}

BT::NodeStatus BallHeldTimerCondition::tick()
{
  double max_s = 8.0;
  std::string area = "own_defense";
  (void)getInput("max_s", max_s);
  (void)getInput("area", area);

  static rclcpp::Clock clock(RCL_STEADY_TIME);
  if (!(max_s > 0.0)) {
    RCLCPP_ERROR_THROTTLE(
      kLogger, clock, 2000,
      "BallHeldTimer: max_s (%.2f) deve ser positivo", max_s);
    return BT::NodeStatus::FAILURE;
  }
  if (area != "own_defense" && area != "opp_defense") {
    RCLCPP_ERROR_THROTTLE(
      kLogger, clock, 2000,
      "BallHeldTimer: area invalida '%s' (esperado own_defense ou opp_defense)", area.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto * bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
  if (!bb) {
    return BT::NodeStatus::FAILURE;
  }

  double my_goal_x = 0.0;
  if (!bb->get("my_goal_x", my_goal_x) || std::abs(my_goal_x) < 1.0) {
    if (!bb->get("own_goal_x", my_goal_x) || std::abs(my_goal_x) < 1.0) {
      RCLCPP_ERROR_THROTTLE(
        kLogger, clock, 2000, "BallHeldTimer: my_goal_x/own_goal_x ausente ou invalido");
      return BT::NodeStatus::FAILURE;
    }
  }

  double area_depth = 0.0;
  double area_width = 0.0;
  if (!bb->get("penalty_area_depth", area_depth) ||
    !bb->get("penalty_area_width", area_width) ||
    area_depth <= 0.0 || area_width <= 0.0)
  {
    RCLCPP_ERROR_THROTTLE(
      kLogger, clock, 2000, "BallHeldTimer: dimensoes da area ausentes ou invalidas");
    return BT::NodeStatus::FAILURE;
  }

  double ball_x = 0.0;
  double ball_y = 0.0;
  if (!bb->get("ball_x", ball_x) || !bb->get("ball_y", ball_y)) {
    RCLCPP_ERROR_THROTTLE(
      kLogger, clock, 2000, "BallHeldTimer: ball_x/ball_y ausentes na blackboard");
    return BT::NodeStatus::FAILURE;
  }

  double ball_age_ms = 0.0;
  if (bb->get("ball_age_ms", ball_age_ms) && ball_age_ms > 500.0) {
    tracking_ = false;
    RCLCPP_WARN_THROTTLE(
      kLogger, clock, 2000, "BallHeldTimer: ball_age_ms=%.1f > 500ms -> FAILURE", ball_age_ms);
    return BT::NodeStatus::FAILURE;
  }

  const double target_goal_x = (area == "own_defense") ? my_goal_x : -my_goal_x;
  if (!isInsidePenaltyArea(ball_x, ball_y, target_goal_x, area_depth, area_width)) {
    tracking_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  double ball_vx = 0.0;
  double ball_vy = 0.0;
  (void)bb->get("ball_vx", ball_vx);
  (void)bb->get("ball_vy", ball_vy);
  const double speed_mm_s = std::hypot(ball_vx, ball_vy);

  const auto now = Clock::now();
  const bool gap = tracking_ &&
    std::chrono::duration<double>(now - last_tick_).count() > kEpisodeGapS;
  const double moved_mm = tracking_ ? std::hypot(ball_x - anchor_x_, ball_y - anchor_y_) : 0.0;

  if (!tracking_ || gap || moved_mm >= kMovedRadiusMm || speed_mm_s > kMovingSpeedMmS) {
    tracking_ = true;
    anchor_x_ = ball_x;
    anchor_y_ = ball_y;
    held_since_ = now;
  }
  last_tick_ = now;

  const double elapsed_s = std::chrono::duration<double>(now - held_since_).count();
  if (elapsed_s >= max_s) {
    RCLCPP_WARN_THROTTLE(
      kLogger, clock, 1000,
      "BallHeldTimer: bola retida em %s por %.1fs (limite %.1fs) -> FAILURE",
      area.c_str(), elapsed_s, max_s);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace oxebots_strategy
