// Leitura da geometria do campo e da pose do robô para os nós do goleiro.
//
// Tudo é lido do blackboard RAIZ (onde o strategy_node escreve /field_geometry e /game_data), e não do
// blackboard da (sub)árvore: assim o resultado não depende de a GoalkeeperTree ser chamada com ou sem
// _autoremap. Chaves ausentes ou inválidas fazem a leitura falhar; o nó que chama deve esperar (e avisar)
// em vez de assumir valores padrão, que poderiam inverter o lado do gol ou usar uma área de outro campo.
#ifndef OXEBOTS_STRATEGY__GOALKEEPER_BLACKBOARD_HPP_
#define OXEBOTS_STRATEGY__GOALKEEPER_BLACKBOARD_HPP_

#include <cmath>

#include "behaviortree_cpp/behavior_tree.h"

namespace oxebots_strategy::goalkeeper
{

// Fração da metade aliada (distância da linha do gol até o meio-campo) até onde o goleiro pode ir buscar a bola.
constexpr double kDefaultLeashFraction = 0.7;

struct FieldGeometry
{
  double my_goal_x = 0.0;
  double field_width = 0.0;
  double penalty_area_depth = 0.0;
  double penalty_area_width = 0.0;
  double goal_width = 0.0;

  // Meio-comprimento do campo (o gol fica em x = +-field_length/2).
  double halfLength() const {return std::abs(my_goal_x);}
  double halfWidth() const {return field_width / 2.0;}
  // +1 se o nosso gol está em -x (o ataque é para +x), -1 caso contrário.
  double forwardSign() const {return my_goal_x < 0.0 ? 1.0 : -1.0;}
};

// rootBlackboard() devolve um ponteiro cru (Blackboard*), não um shared_ptr; nullptr se o nó não tem blackboard.
inline BT::Blackboard * rootBlackboard(const BT::NodeConfig & config)
{
  return config.blackboard ? config.blackboard->rootBlackboard() : nullptr;
}

// Falha se alguma chave faltar ou ainda estiver zerada (o strategy_node inicializa tudo com 0.0 até chegar
// a primeira /field_geometry).
inline bool readFieldGeometry(const BT::NodeConfig & config, FieldGeometry & geometry)
{
  const auto root = rootBlackboard(config);
  if (!root) {
    return false;
  }
  FieldGeometry g;
  if (!root->get("my_goal_x", g.my_goal_x) ||
    !root->get("field_width", g.field_width) ||
    !root->get("penalty_area_depth", g.penalty_area_depth) ||
    !root->get("penalty_area_width", g.penalty_area_width) ||
    !root->get("goal_width", g.goal_width))
  {
    return false;
  }
  if (std::abs(g.my_goal_x) < 1.0 || g.field_width <= 0.0 || g.penalty_area_depth <= 0.0 ||
    g.penalty_area_width <= 0.0 || g.goal_width <= 0.0)
  {
    return false;
  }
  geometry = g;
  return true;
}

inline bool readRobotPose(const BT::NodeConfig & config, double & x, double & y, double & yaw)
{
  const auto root = rootBlackboard(config);
  return root && root->get("robot_x", x) && root->get("robot_y", y) && root->get("robot_yaw", yaw);
}

}  // namespace oxebots_strategy::goalkeeper

#endif  // OXEBOTS_STRATEGY__GOALKEEPER_BLACKBOARD_HPP_
