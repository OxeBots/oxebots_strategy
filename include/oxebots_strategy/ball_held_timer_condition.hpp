#pragma once

#include <chrono>
#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace oxebots_strategy
{

// Regra dos 10 s da SSL (§8.3.1, R-19, SC-25): devolve FAILURE quando a bola permanece
// parada dentro da área monitorada ("own_defense" ou "opp_defense") por pelo menos `max_s`
// segundos (padrão 8.0 s, dando 2 s de margem antes dos 10 s regulamentares), e SUCCESS
// enquanto a bola está fora da área, em movimento ou abaixo do limite de tempo.
// Sem geometria ou dados da bola na blackboard raiz, devolve FAILURE com log.
class BallHeldTimerCondition : public BT::ConditionNode
{
public:
  BallHeldTimerCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  static constexpr double kEpisodeGapS = 0.5;
  static constexpr double kMovedRadiusMm = 80.0;
  static constexpr double kMovingSpeedMmS = 150.0;

private:
  using Clock = std::chrono::steady_clock;

  bool tracking_ = false;
  double anchor_x_ = 0.0;
  double anchor_y_ = 0.0;
  Clock::time_point held_since_;
  Clock::time_point last_tick_;
};

}  // namespace oxebots_strategy
