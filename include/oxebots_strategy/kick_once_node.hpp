#pragma once

#include <string>
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_motion_override.hpp"

namespace oxebots_strategy
{

// Chuta UMA vez e confirma o RESULTADO (E08, Capítulos 18 e 21).
//
// Diferença para o KickBall: o KickBall termina quando o controlador reporta que o disparo
// aconteceu; o KickOnce só devolve SUCCESS quando a bola de fato se moveu pelo menos
// `ball_moved_mm` desde o pedido (é a regra de "bola em jogo" do regulamento, §5.4), e carrega o
// prazo de `timeout_s` (10 s no regulamento) devolvendo FAILURE se a bola não se mexer.
//
// O disparo físico (avanço + kick_speed, janela de contato, cooldown) continua no controlador
// central: este nó só pede MODE_KICK em /robot_motion_override a cada tick e libera o override
// (MODE_NONE) ao terminar ou ao ser interrompido. Pré-condição: o robô já está encostado na bola
// e alinhado (o controlador só avança por ~100 mm durante o disparo); quem garante isso é a
// árvore que chama (RestartKickerTree).
//
// A posição da bola vem da blackboard raiz (ball_x/ball_y, em mm), mantida pelo strategy_node.
class KickOnceNode : public BT::StatefulActionNode
{
public:
  KickOnceNode(const std::string & name, const BT::NodeConfig & config,
               rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  // Teto da velocidade do chute (regra R-25: bola acelerada a no máximo 3 m/s no SSL-EL). O
  // controlador também limita (SafetyGate, regra 4); este clamp evita pedir o que seria cortado.
  static constexpr double kMaxKickSpeed = 3.0;
  static constexpr double kDefaultKickSpeed = 2.5;

private:
  BT::NodeStatus step();
  bool readBall(double & x, double & y) const;
  void requestKick();
  void releaseOverride();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotMotionOverride>::SharedPtr override_pub_;

  unsigned int robot_id_ = 0;
  double kick_speed_ = 0.0;
  double timeout_s_ = 10.0;
  double ball_moved_mm_ = 50.0;
  double ball_start_x_ = 0.0;
  double ball_start_y_ = 0.0;
  rclcpp::Time start_time_;
};

}  // namespace oxebots_strategy
