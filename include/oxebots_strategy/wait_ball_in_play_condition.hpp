#pragma once

#include <chrono>
#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace oxebots_strategy
{

// Regra "bola em jogo" do regulamento (§5.4, R-16): SUCCESS quando a bola se moveu pelo menos
// `moved_mm` desde o início do reinício OU passaram `timeout_s`; FAILURE enquanto ainda se espera.
// (O nome vem do uso: "esperar a bola entrar em jogo"; embrulhado em Inverter vira "enquanto
// espera".) Usada pelas folhas que precisam reagir DENTRO de uma fase de reinício, como o goleiro
// do pênalti, que só pode sair da linha depois que a bola entra em jogo.
//
// "Início do reinício" = a primeira vez que a condição é avaliada num EPISÓDIO. Um novo episódio
// começa quando (a) é a primeira avaliação, (b) a fase da blackboard mudou ou (c) a condição ficou
// mais de kEpisodeGapS sem ser avaliada (o ramo foi interrompido e voltou depois). A posição de
// referência da bola e o relógio são registrados nesse instante. Passada a bola em jogo o resultado
// fica travado em SUCCESS até o próximo episódio (a bola voltar perto da referência não "desfaz").
//
// Lê da blackboard raiz: ball_x, ball_y (mm) e phase (int, opcional). Sem a bola, FAILURE com log.
class WaitBallInPlayCondition : public BT::ConditionNode
{
public:
  WaitBallInPlayCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  // Sem avaliação por mais que isto, a próxima avaliação inicia um episódio novo.
  static constexpr double kEpisodeGapS = 0.5;

private:
  using Clock = std::chrono::steady_clock;

  bool has_episode_ = false;
  bool in_play_ = false;
  int last_phase_ = -1;
  double ref_x_ = 0.0;
  double ref_y_ = 0.0;
  Clock::time_point episode_start_;
  Clock::time_point last_tick_;
};

}  // namespace oxebots_strategy
