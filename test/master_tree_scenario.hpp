// Cenário (blackboard "mock") da Master Tree: as chaves que as guardas e as folhas dos ramos da
// MainTreeEL leem, escritas com os MESMOS nomes e tipos que o StrategyNode::refresh() escreve.
//
// O teste test_strategy_node.cpp (MasterTreeBlackboardContract) confere que o refresh() produz todas
// estas chaves com estes tipos; assim o mock não diverge do nó real sem alguém perceber.
#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <vector>

#include "behaviortree_cpp/blackboard.h"
#include "oxebots_interfaces/msg/game_state.hpp"
#include "oxebots_strategy/defender_nodes.hpp"  // Pose2D, Vector2D

namespace master_tree_test
{

using GameState = oxebots_interfaces::msg::GameState;

// Fases como o refresh() as grava na blackboard (int do GameState.phase).
constexpr int kHalt = GameState::PHASE_HALT;
constexpr int kTimeout = GameState::PHASE_TIMEOUT;
constexpr int kStop = GameState::PHASE_STOP;
constexpr int kBallPlacement = GameState::PHASE_BALL_PLACEMENT;
constexpr int kPrepareKickoff = GameState::PHASE_PREPARE_KICKOFF;
constexpr int kKickoff = GameState::PHASE_KICKOFF;
constexpr int kPreparePenalty = GameState::PHASE_PREPARE_PENALTY;
constexpr int kPenalty = GameState::PHASE_PENALTY;
constexpr int kFreeKick = GameState::PHASE_FREE_KICK;
constexpr int kRun = GameState::PHASE_RUN;

struct Scenario
{
  // --- Estado de jogo (GameState) ---
  int phase{kRun};
  bool we_favored{false};
  bool require_gc{true};
  bool has_designated{false};
  double designated_x{0.0};
  double designated_y{0.0};

  // --- Idade dos dados (ms). Dentro dos limites padrão: 500 / 150 / 300 ---
  double gc_age_ms{10.0};
  double pose_age_ms{10.0};
  double ball_age_ms{10.0};
  double max_gc_ms{500.0};
  double max_pose_ms{150.0};
  double max_ball_ms{300.0};

  // --- Papel (RoleAssignment) ---
  uint32_t robot_id{1};
  std::string role{"idle"};  // goalkeeper | attacker | defender | idle
  bool is_kicker{false};
  uint32_t kicker_id{999};  // NO_ROBOT
  bool has_defender{true};  // P03: falso com só goleiro + atacante em campo

  // --- Cor e lado ---
  bool is_yellow{false};
  // +1: defendemos o gol de x negativo (own_goal_x = -L/2); -1: defendemos x positivo.
  int side_sign{1};
  double field_length{4500.0};
  double field_width{3000.0};

  // --- Mundo ---
  double ball_x{0.0};
  double ball_y{0.0};
  double robot_x{-1200.0};
  double robot_y{0.0};
  double robot_yaw{0.0};

  double ownGoalX() const {return side_sign > 0 ? -field_length / 2.0 : field_length / 2.0;}

  void applyTo(BT::Blackboard & bb) const
  {
    bb.set("phase", phase);
    bb.set("we_favored", we_favored);
    bb.set("require_gc", require_gc);
    bb.set("has_designated", has_designated);
    bb.set("designated_x", designated_x);
    bb.set("designated_y", designated_y);

    bb.set("gc_age_ms", gc_age_ms);
    bb.set("pose_age_ms", pose_age_ms);
    bb.set("ball_age_ms", ball_age_ms);
    bb.set("max_gc_ms", max_gc_ms);
    bb.set("max_pose_ms", max_pose_ms);
    bb.set("max_ball_ms", max_ball_ms);

    bb.set("robot_id", robot_id);
    bb.set("role", role);
    bb.set("is_kicker", is_kicker);
    bb.set("kicker_id", kicker_id);
    bb.set("has_defender", has_defender);

    bb.set("is_yellow", is_yellow);
    bb.set("side_sign", side_sign);
    bb.set("field_length", field_length);
    bb.set("field_width", field_width);
    bb.set("own_goal_x", ownGoalX());
    bb.set("opp_goal_x", -ownGoalX());
    bb.set("opp_goal_y", 0.0);

    bb.set("ball_x", ball_x);
    bb.set("ball_y", ball_y);
    bb.set("ball", Pose2D{ball_x, ball_y, 0.0});
    bb.set("robot_x", robot_x);
    bb.set("robot_y", robot_y);
    bb.set("robot_yaw", robot_yaw);
    bb.set("robot_pose", Pose2D{robot_x, robot_y, robot_yaw});
  }
};

// Chaves que o Scenario escreve e o tipo de cada uma. O teste de contrato compara com o refresh().
inline const std::map<std::string, std::type_index> & expectedKeyTypes()
{
  static const std::map<std::string, std::type_index> types{
    {"phase", typeid(int)},
    {"we_favored", typeid(bool)},
    {"require_gc", typeid(bool)},
    {"has_designated", typeid(bool)},
    {"designated_x", typeid(double)},
    {"designated_y", typeid(double)},
    {"gc_age_ms", typeid(double)},
    {"pose_age_ms", typeid(double)},
    {"ball_age_ms", typeid(double)},
    {"max_gc_ms", typeid(double)},
    {"max_pose_ms", typeid(double)},
    {"max_ball_ms", typeid(double)},
    {"robot_id", typeid(uint32_t)},
    {"role", typeid(std::string)},
    {"is_kicker", typeid(bool)},
    {"kicker_id", typeid(uint32_t)},
    {"has_defender", typeid(bool)},
    {"is_yellow", typeid(bool)},
    {"side_sign", typeid(int)},
    {"field_length", typeid(double)},
    {"field_width", typeid(double)},
    {"own_goal_x", typeid(double)},
    {"opp_goal_x", typeid(double)},
    {"opp_goal_y", typeid(double)},
    {"ball_x", typeid(double)},
    {"ball_y", typeid(double)},
    {"ball", typeid(Pose2D)},
    {"robot_x", typeid(double)},
    {"robot_y", typeid(double)},
    {"robot_yaw", typeid(double)},
    {"robot_pose", typeid(Pose2D)},
  };
  return types;
}

}  // namespace master_tree_test
