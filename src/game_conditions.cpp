#include "oxebots_strategy/game_conditions.hpp"

#include "rclcpp/rclcpp.hpp"

#include <string>

namespace oxebots_strategy
{
namespace
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("game_conditions");

BT::NodeStatus boolStatus(bool value)
{
  return value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void logError(const char * message)
{
  static rclcpp::Clock clock;
  RCLCPP_ERROR_THROTTLE(LOGGER, clock, 1000, "%s", message);
}

void logWarning(const char * message)
{
  static rclcpp::Clock clock;
  RCLCPP_WARN_THROTTLE(LOGGER, clock, 1000, "%s", message);
}

bool readString(const BT::Blackboard::Ptr & blackboard, const char * key, std::string & value)
{
  return blackboard && blackboard->get(key, value);
}

bool readPhase(const BT::Blackboard::Ptr & blackboard, std::string & phase)
{
  if (readString(blackboard, "phase", phase)) {
    return true;
  }

  int phase_id;
  if (!blackboard || !blackboard->get("phase", phase_id) ||
      phase_id < 0 || phase_id > 9) {
    return false;
  }

  static const char * const phase_names[] = {
    "HALT", "TIMEOUT", "STOP", "BALL_PLACEMENT", "PREPARE_KICKOFF",
    "KICKOFF", "PREPARE_PENALTY", "PENALTY", "FREE_KICK", "RUN"};
  phase = phase_names[phase_id];
  return true;
}

}  // namespace

CheckGameStateCondition::CheckGameStateCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckGameStateCondition::providedPorts()
{
  return {BT::InputPort<std::string>("phase")};
}

BT::NodeStatus CheckGameStateCondition::tick()
{
  std::string expected;
  if (!getInput("phase", expected) || expected.empty()) {
    logError("CheckGameState: porta [phase] ausente ou vazia");
    return BT::NodeStatus::FAILURE;
  }

  std::string current;
  if (!readPhase(config().blackboard, current)) {
    logError("CheckGameState: chave [phase] ausente ou inválida na Blackboard");
    return BT::NodeStatus::FAILURE;
  }

  return boolStatus(current == expected);
}

WeAreFavoredCondition::WeAreFavoredCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList WeAreFavoredCondition::providedPorts()
{
  return {};
}

BT::NodeStatus WeAreFavoredCondition::tick()
{
  bool favored = false;
  if (!config().blackboard || !config().blackboard->get("we_favored", favored)) {
    logError("WeAreFavored: chave [we_favored] ausente na Blackboard");
    return BT::NodeStatus::FAILURE;
  }

  return boolStatus(favored);
}

IsDataFreshCondition::IsDataFreshCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList IsDataFreshCondition::providedPorts()
{
  return {
    BT::InputPort<double>("max_gc_ms", 500.0),
    BT::InputPort<double>("max_pose_ms", 150.0),
    BT::InputPort<double>("max_ball_ms", 300.0)};
}

BT::NodeStatus IsDataFreshCondition::tick()
{
  double max_gc_ms;
  double max_pose_ms;
  double max_ball_ms;
  if (!getInput("max_gc_ms", max_gc_ms) ||
      !getInput("max_pose_ms", max_pose_ms) ||
      !getInput("max_ball_ms", max_ball_ms) ||
      max_gc_ms < 0.0 || max_pose_ms < 0.0 || max_ball_ms < 0.0) {
    logError("IsDataFresh: portas de limite ausentes ou inválidas");
    return BT::NodeStatus::FAILURE;
  }

  double gc_age_ms;
  double pose_age_ms;
  double ball_age_ms;
  const auto blackboard = config().blackboard;
  if (!blackboard ||
      !blackboard->get("gc_age_ms", gc_age_ms) ||
      !blackboard->get("pose_age_ms", pose_age_ms) ||
      !blackboard->get("ball_age_ms", ball_age_ms)) {
    logError("IsDataFresh: uma ou mais idades estão ausentes na Blackboard");
    return BT::NodeStatus::FAILURE;
  }

  const bool gc_fresh = gc_age_ms <= max_gc_ms;
  const bool pose_fresh = pose_age_ms <= max_pose_ms;
  const bool ball_fresh = ball_age_ms <= max_ball_ms;
  if (!gc_fresh || !pose_fresh || !ball_fresh) {
    if (!gc_fresh) {
      logWarning("IsDataFresh: gc_age_ms excedeu max_gc_ms");
    }
    if (!pose_fresh) {
      logWarning("IsDataFresh: pose_age_ms excedeu max_pose_ms");
    }
    if (!ball_fresh) {
      logWarning("IsDataFresh: ball_age_ms excedeu max_ball_ms");
    }
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

HasRoleCondition::HasRoleCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList HasRoleCondition::providedPorts()
{
  return {BT::InputPort<std::string>("role")};
}

BT::NodeStatus HasRoleCondition::tick()
{
  std::string expected_role;
  if (!getInput("role", expected_role) || expected_role.empty()) {
    logError("HasRole: porta [role] ausente ou vazia");
    return BT::NodeStatus::FAILURE;
  }

  std::string current_role;
  if (!readString(config().blackboard, "role", current_role)) {
    logError("HasRole: chave [role] ausente na Blackboard");
    return BT::NodeStatus::FAILURE;
  }

  return boolStatus(current_role == expected_role);
}

}  // namespace oxebots_strategy
