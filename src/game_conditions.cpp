#include "oxebots_strategy/game_conditions.hpp"

#include "rclcpp/rclcpp.hpp"

#include <cmath>
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

bool isValidPhaseName(const std::string & phase)
{
  static const char * const kValidPhases[] = {
    "HALT", "TIMEOUT", "STOP", "BALL_PLACEMENT", "PREPARE_KICKOFF",
    "KICKOFF", "PREPARE_PENALTY", "PENALTY", "FREE_KICK", "RUN"};
  for (const char * valid : kValidPhases) {
    if (phase == valid) {
      return true;
    }
  }
  return false;
}

bool isValidRoleName(const std::string & role)
{
  static const char * const kValidRoles[] = {
    "goalkeeper", "attacker", "defender", "kicker", "idle"};
  for (const char * valid : kValidRoles) {
    if (role == valid) {
      return true;
    }
  }
  return false;
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
    if (bb->get("phase", phase_id)) {
      if (phase_id >= 0 && phase_id <= 9) {
        static const char * const phase_names[] = {
          "HALT", "TIMEOUT", "STOP", "BALL_PLACEMENT", "PREPARE_KICKOFF",
          "KICKOFF", "PREPARE_PENALTY", "PENALTY", "FREE_KICK", "RUN"};
        phase = phase_names[phase_id];
        return true;
      }
      return false;
    }
  } catch (const std::exception &) {
    // Valor armazenado não é conversível para int — tenta string abaixo
  }

  // Fallback: alguém escreveu a fase como string diretamente
  return bb->get("phase", phase) && isValidPhaseName(phase);
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
  if (!getInput("phase", expected) || !isValidPhaseName(expected)) {
    logError("CheckGameState: porta [phase] ausente ou inválida");
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
    BT::InputPort<double>(
      "max_gc_ms", "Opcional: idade máxima do Game Controller em ms (padrão 500.0)"),
    BT::InputPort<double>(
      "max_pose_ms", "Opcional: idade máxima da pose em ms (padrão 150.0)"),
    BT::InputPort<double>(
      "max_ball_ms", "Opcional: idade máxima da bola em ms (padrão 300.0)"),
    BT::InputPort<bool>(
      "require_gc", "Opcional: se falso, ignora a verificação de idade do GC")};
}

BT::NodeStatus IsDataFreshCondition::tick()
{
  auto * bb = rootBB(config());
  if (!bb) {
    logError("IsDataFresh: Blackboard inexistente");
    return BT::NodeStatus::FAILURE;
  }

  // Ordem de precedência para os limites:
  // 1. Valores padrão (500 / 150 / 300 ms)
  // 2. Blackboard (parâmetros YAML carregados pelo strategy_node)
  // 3. Portas explícitas no XML da BT (maior prioridade)
  double max_gc_ms = 500.0;
  double max_pose_ms = 150.0;
  double max_ball_ms = 300.0;
  (void)bb->get("max_gc_ms", max_gc_ms);
  (void)bb->get("max_pose_ms", max_pose_ms);
  (void)bb->get("max_ball_ms", max_ball_ms);

  const auto & ports = config().input_ports;
  if (ports.find("max_gc_ms") != ports.end() && !getInput("max_gc_ms", max_gc_ms)) {
    logError("IsDataFresh: porta [max_gc_ms] inválida");
    return BT::NodeStatus::FAILURE;
  }
  if (ports.find("max_pose_ms") != ports.end() && !getInput("max_pose_ms", max_pose_ms)) {
    logError("IsDataFresh: porta [max_pose_ms] inválida");
    return BT::NodeStatus::FAILURE;
  }
  if (ports.find("max_ball_ms") != ports.end() && !getInput("max_ball_ms", max_ball_ms)) {
    logError("IsDataFresh: porta [max_ball_ms] inválida");
    return BT::NodeStatus::FAILURE;
  }

  if (!std::isfinite(max_gc_ms) || !std::isfinite(max_pose_ms) || !std::isfinite(max_ball_ms) ||
      max_gc_ms < 0.0 || max_pose_ms < 0.0 || max_ball_ms < 0.0) {
    logError("IsDataFresh: limites de idade negativos ou inválidos");
    return BT::NodeStatus::FAILURE;
  }

  // Determina se o GC é obrigatório:
  // 1. Valor padrão: true
  // 2. Blackboard (via parâmetro YAML no strategy_node: 'require_gc' ou 'use_gc')
  // 3. Porta 'require_gc' explícita no XML da BT (maior prioridade)
  bool require_gc = true;
  if (!bb->get("require_gc", require_gc)) {
    (void)bb->get("use_gc", require_gc);
  }
  if (ports.find("require_gc") != ports.end() && !getInput("require_gc", require_gc)) {
    logError("IsDataFresh: porta [require_gc] inválida");
    return BT::NodeStatus::FAILURE;
  }

  double pose_age_ms;
  double ball_age_ms;
  if (!bb->get("pose_age_ms", pose_age_ms) ||
      !bb->get("ball_age_ms", ball_age_ms)) {
    logError("IsDataFresh: pose_age_ms ou ball_age_ms ausentes na Blackboard");
    return BT::NodeStatus::FAILURE;
  }

  bool gc_fresh = true;
  if (require_gc && max_gc_ms > 0.0) {
    double gc_age_ms;
    if (!bb->get("gc_age_ms", gc_age_ms)) {
      logError("IsDataFresh: gc_age_ms ausente na Blackboard e require_gc=true");
      return BT::NodeStatus::FAILURE;
    }
    gc_fresh = std::isfinite(gc_age_ms) && (gc_age_ms >= 0.0) && (gc_age_ms <= max_gc_ms);
    if (!gc_fresh) {
      logWarning("IsDataFresh: gc_age_ms excedeu max_gc_ms");
    }
  }

  const bool pose_fresh =
    std::isfinite(pose_age_ms) && (pose_age_ms >= 0.0) && (pose_age_ms <= max_pose_ms);
  const bool ball_fresh =
    std::isfinite(ball_age_ms) && (ball_age_ms >= 0.0) && (ball_age_ms <= max_ball_ms);
  if (!pose_fresh) {
    logWarning("IsDataFresh: pose_age_ms excedeu max_pose_ms");
  }
  if (!ball_fresh) {
    logWarning("IsDataFresh: ball_age_ms excedeu max_ball_ms");
  }

  if (!gc_fresh || !pose_fresh || !ball_fresh) {
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
  if (!getInput("role", expected_role) || !isValidRoleName(expected_role)) {
    logError("HasRole: porta [role] ausente ou inválida");
    return BT::NodeStatus::FAILURE;
  }

  std::string current_role;
  if (!readString(rootBB(config()), "role", current_role) || !isValidRoleName(current_role)) {
    logError("HasRole: chave [role] ausente ou inválida na Blackboard");
    return BT::NodeStatus::FAILURE;
  }

  return boolStatus(current_role == expected_role);
}

}  // namespace oxebots_strategy
