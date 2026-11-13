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

// Na BT.CPP 4.x, portas com valor padrão em providedPorts() são pré-preenchidas
// em config().input_ports pelo parser XML com std::to_string(default_value)
// (ex.: "150.000000") mesmo quando omitidas na tag XML. Esta função distingue
// um atributo explícito no XML do valor padrão injetado automaticamente.
bool hasExplicitPort(
  const BT::PortsRemapping & ports, const char * port_name, double default_value)
{
  const auto it = ports.find(port_name);
  return it != ports.end() && it->second != std::to_string(default_value);
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
    BT::InputPort<double>("max_gc_ms", 500.0, "Idade máxima do Game Controller em ms"),
    BT::InputPort<double>("max_pose_ms", 150.0, "Idade máxima da pose em ms"),
    BT::InputPort<double>("max_ball_ms", 300.0, "Idade máxima da bola em ms"),
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
  // 1. Valores padrão das portas (500 / 150 / 300 ms)
  // 2. Blackboard (parâmetros YAML carregados pelo strategy_node)
  // 3. Portas explícitas no XML da BT (maior prioridade)
  double max_gc_ms = 500.0;
  double max_pose_ms = 150.0;
  double max_ball_ms = 300.0;
  (void)bb->get("max_gc_ms", max_gc_ms);
  (void)bb->get("max_pose_ms", max_pose_ms);
  (void)bb->get("max_ball_ms", max_ball_ms);

  const auto & ports = config().input_ports;
  if (hasExplicitPort(ports, "max_gc_ms", 500.0) && !getInput("max_gc_ms", max_gc_ms)) {
    logError("IsDataFresh: porta [max_gc_ms] inválida");
    return BT::NodeStatus::FAILURE;
  }
  if (hasExplicitPort(ports, "max_pose_ms", 150.0) && !getInput("max_pose_ms", max_pose_ms)) {
    logError("IsDataFresh: porta [max_pose_ms] inválida");
    return BT::NodeStatus::FAILURE;
  }
  if (hasExplicitPort(ports, "max_ball_ms", 300.0) && !getInput("max_ball_ms", max_ball_ms)) {
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
  auto * bb = rootBB(config());
  if (!readString(bb, "role", current_role) || !isValidRoleName(current_role)) {
    logError("HasRole: chave [role] ausente ou inválida na Blackboard");
    return BT::NodeStatus::FAILURE;
  }

  if (current_role == expected_role) {
    return BT::NodeStatus::SUCCESS;
  }

  if (expected_role == "kicker" && bb) {
    bool is_kicker = false;
    if (bb->get("is_kicker", is_kicker) && is_kicker) {
      return BT::NodeStatus::SUCCESS;
    }
    uint32_t my_id = 0, kicker_id = 999;
    if (bb->get("robot_id", my_id) && bb->get("kicker_id", kicker_id) && my_id == kicker_id) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}

IsCornerKickCondition::IsCornerKickCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList IsCornerKickCondition::providedPorts()
{
  return {
    BT::InputPort<double>("x_margin", 750.0, "Margem máxima em X até a linha de fundo adversária (mm)"),
    BT::InputPort<double>("y_margin", 650.0, "Margem máxima em Y até a linha lateral (mm)")
  };
}

BT::NodeStatus IsCornerKickCondition::tick()
{
  auto * bb = rootBB(config());
  if (!bb) {
    return BT::NodeStatus::FAILURE;
  }

  double x_margin = 750.0;
  double y_margin = 650.0;
  getInput("x_margin", x_margin);
  getInput("y_margin", y_margin);

  double ball_x = 0.0;
  double ball_y = 0.0;
  if (!bb->get("ball_x", ball_x) || !bb->get("ball_y", ball_y)) {
    return BT::NodeStatus::FAILURE;
  }

  double field_length = 4500.0;
  if (!bb->get("field_length", field_length) || field_length <= 1.0) {
    field_length = 4500.0;
  }
  double field_width = 3000.0;
  if (!bb->get("field_width", field_width) || field_width <= 1.0) {
    field_width = 3000.0;
  }

  double opp_goal_x = 0.0;
  if (!bb->get("opp_goal_x", opp_goal_x) || std::abs(opp_goal_x) < 1.0) {
    double own_goal_x = 0.0;
    if (bb->get("own_goal_x", own_goal_x) && std::abs(own_goal_x) > 1.0) {
      opp_goal_x = -own_goal_x;
    } else if (bb->get("my_goal_x", own_goal_x) && std::abs(own_goal_x) > 1.0) {
      opp_goal_x = -own_goal_x;
    } else {
      int side_sign = 0;
      if (bb->get("side_sign", side_sign) && side_sign != 0) {
        opp_goal_x = (side_sign > 0) ? (field_length / 2.0) : -(field_length / 2.0);
      } else {
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  const double attack_sign = (opp_goal_x > 0.0) ? 1.0 : -1.0;
  const bool in_offensive_half = (ball_x * attack_sign) > 0.0;
  const bool near_goal_line = std::abs(opp_goal_x - ball_x) <= x_margin;
  const bool near_sideline = std::abs(ball_y) >= (field_width / 2.0 - y_margin);

  return boolStatus(in_offensive_half && near_goal_line && near_sideline);
}

IsThrowInCondition::IsThrowInCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList IsThrowInCondition::providedPorts()
{
  return {
    BT::InputPort<double>("y_margin", 450.0, "Margem máxima em Y até a linha lateral (mm)"),
    BT::InputPort<double>("corner_x_margin", 750.0, "Margem em X das linhas de fundo para excluir escanteio/tiro de meta (mm)")
  };
}

BT::NodeStatus IsThrowInCondition::tick()
{
  auto * bb = rootBB(config());
  if (!bb) {
    return BT::NodeStatus::FAILURE;
  }

  double y_margin = 450.0;
  double corner_x_margin = 750.0;
  getInput("y_margin", y_margin);
  getInput("corner_x_margin", corner_x_margin);

  double ball_x = 0.0;
  double ball_y = 0.0;
  if (!bb->get("ball_x", ball_x) || !bb->get("ball_y", ball_y)) {
    return BT::NodeStatus::FAILURE;
  }

  double field_length = 4500.0;
  if (!bb->get("field_length", field_length) || field_length <= 1.0) {
    field_length = 4500.0;
  }
  double field_width = 3000.0;
  if (!bb->get("field_width", field_width) || field_width <= 1.0) {
    field_width = 3000.0;
  }

  const bool near_sideline = std::abs(ball_y) >= (field_width / 2.0 - y_margin);
  const bool outside_corners = std::abs(ball_x) < (field_length / 2.0 - corner_x_margin);

  return boolStatus(near_sideline && outside_corners);
}

}  // namespace oxebots_strategy
