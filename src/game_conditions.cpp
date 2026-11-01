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

// Acesso seguro à blackboard raiz — as chaves de estado (phase, we_favored,
// gc_age_ms, role…) são escritas pelo strategy_node na raiz, e dentro de
// SubTrees a blackboard local pode não tê-las.  Padrão do projeto
// (goalkeeper_node, go_to_point_node, defender_nodes).
BT::Blackboard * rootBB(const BT::NodeConfig & config)
{
  return config.blackboard ? config.blackboard->rootBlackboard() : nullptr;
}

bool readString(BT::Blackboard * bb, const char * key, std::string & value)
{
  return bb && bb->get(key, value);
}

bool readPhase(BT::Blackboard * bb, std::string & phase)
{
  if (!bb) {
    return false;
  }

  // Tenta int primeiro — é o tipo real escrito pelo strategy_node.
  // Se o valor armazenado for string (ex.: "STOP"), a conversão para int
  // lança exceção na BT.CPP 4.x; nesse caso caímos no fallback.
  try {
    int phase_id = -1;
    if (bb->get("phase", phase_id) && phase_id >= 0 && phase_id <= 9) {
      static const char * const phase_names[] = {
        "HALT", "TIMEOUT", "STOP", "BALL_PLACEMENT", "PREPARE_KICKOFF",
        "KICKOFF", "PREPARE_PENALTY", "PENALTY", "FREE_KICK", "RUN"};
      phase = phase_names[phase_id];
      return true;
    }
  } catch (const std::exception &) {
    // Valor armazenado não é conversível para int — tenta string abaixo
  }

  // Fallback: alguém escreveu a fase como string diretamente
  return bb->get("phase", phase);
}

}  // namespace

CheckGameStateCondition::CheckGameStateCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckGameStateCondition::providedPorts()
{
  return {BT::InputPort<std::string>(
    "phase", "Fase esperada: HALT, TIMEOUT, STOP, BALL_PLACEMENT, "
             "PREPARE_KICKOFF, KICKOFF, PREPARE_PENALTY, PENALTY, FREE_KICK ou RUN")};
}

BT::NodeStatus CheckGameStateCondition::tick()
{
  std::string expected;
  if (!getInput("phase", expected) || expected.empty()) {
    logError("CheckGameState: porta [phase] ausente ou vazia");
    return BT::NodeStatus::FAILURE;
  }

  std::string current;
  if (!readPhase(rootBB(config()), current)) {
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
  auto * bb = rootBB(config());
  if (!bb || !bb->get("we_favored", favored)) {
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
    BT::InputPort<double>("max_gc_ms", 500.0, "Idade máxima do Game Controller em ms"),
    BT::InputPort<double>("max_pose_ms", 150.0, "Idade máxima da pose em ms"),
    BT::InputPort<double>("max_ball_ms", 300.0, "Idade máxima da bola em ms")};
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
  auto * bb = rootBB(config());
  if (!bb ||
      !bb->get("gc_age_ms", gc_age_ms) ||
      !bb->get("pose_age_ms", pose_age_ms) ||
      !bb->get("ball_age_ms", ball_age_ms)) {
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
  return {BT::InputPort<std::string>(
    "role", "Papel esperado: goalkeeper, attacker, defender ou kicker")};
}

BT::NodeStatus HasRoleCondition::tick()
{
  std::string expected_role;
  if (!getInput("role", expected_role) || expected_role.empty()) {
    logError("HasRole: porta [role] ausente ou vazia");
    return BT::NodeStatus::FAILURE;
  }

  std::string current_role;
  if (!readString(rootBB(config()), "role", current_role)) {
    logError("HasRole: chave [role] ausente na Blackboard");
    return BT::NodeStatus::FAILURE;
  }

  return boolStatus(current_role == expected_role);
}

}  // namespace oxebots_strategy
