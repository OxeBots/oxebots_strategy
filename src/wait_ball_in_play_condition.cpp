#include "oxebots_strategy/wait_ball_in_play_condition.hpp"

#include <cmath>

#include "rclcpp/rclcpp.hpp"

namespace oxebots_strategy
{

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("wait_ball_in_play");
}  // namespace

WaitBallInPlayCondition::WaitBallInPlayCondition(const std::string & name,
                                                 const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList WaitBallInPlayCondition::providedPorts()
{
  return {
    BT::InputPort<double>("moved_mm", 50.0, "Deslocamento da bola (mm) que a põe em jogo"),
    BT::InputPort<double>("timeout_s", 10.0, "Tempo (s) após o qual a bola é considerada em jogo")};
}

BT::NodeStatus WaitBallInPlayCondition::tick()
{
  double moved_mm = 50.0;
  double timeout_s = 10.0;
  (void)getInput("moved_mm", moved_mm);
  (void)getInput("timeout_s", timeout_s);
  if (!(moved_mm > 0.0) || !(timeout_s > 0.0)) {
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_ERROR_THROTTLE(kLogger, clock, 2000,
                          "WaitBallInPlay: moved_mm (%.1f) e timeout_s (%.1f) devem ser positivos",
                          moved_mm, timeout_s);
    return BT::NodeStatus::FAILURE;
  }

  auto * bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
  double ball_x = 0.0;
  double ball_y = 0.0;
  if (!bb || !bb->get("ball_x", ball_x) || !bb->get("ball_y", ball_y)) {
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_ERROR_THROTTLE(kLogger, clock, 2000,
                          "WaitBallInPlay: ball_x/ball_y ausentes na blackboard");
    return BT::NodeStatus::FAILURE;
  }

  // A fase é opcional: sem ela o episódio só se reinicia pela lacuna de avaliação.
  int phase = -1;
  if (bb) {
    try {
      (void)bb->get("phase", phase);
    } catch (const std::exception &) {
      phase = -1;
    }
  }

  const auto now = Clock::now();
  const bool gap = has_episode_ &&
    std::chrono::duration<double>(now - last_tick_).count() > kEpisodeGapS;
  if (!has_episode_ || gap || phase != last_phase_) {
    has_episode_ = true;
    in_play_ = false;
    ref_x_ = ball_x;
    ref_y_ = ball_y;
    episode_start_ = now;
  }
  last_tick_ = now;
  last_phase_ = phase;

  if (!in_play_) {
    const double moved = std::hypot(ball_x - ref_x_, ball_y - ref_y_);
    const double elapsed = std::chrono::duration<double>(now - episode_start_).count();
    if (moved >= moved_mm || elapsed >= timeout_s) {
      in_play_ = true;
    }
  }

  return in_play_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace oxebots_strategy
