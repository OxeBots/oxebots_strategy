#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/ball_prediction.hpp"
#include "oxebots_interfaces/msg/robot_prediction.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <mutex>
#include <optional>

namespace oxebots_strategy
{

class CalculateInterceptionNode : public BT::StatefulActionNode
{
public:
  CalculateInterceptionNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  // Distância (mm) do ponto de captura até a bola prevista: perto o bastante para o dribbler
  // encostar já alinhado, sem mirar no centro exato da bola (evita colisão de frente).
  static constexpr double kCaptureDistanceMm = 115.0;

  // Valor DEFAULT da porta "carry_distance_mm" (ver providedPorts()) — não mais um valor fixo:
  // cada chamada de CalculateInterception pode passar seu próprio carry_distance_mm, então
  // trees/estratégias diferentes podem escolher sua própria agressividade de carga sem editar
  // esta constante. Usado como base do "ponto de carga" (intercept_carry_x/y), só relevante pra
  // árvores que usam esse output (hoje, master_strategy_carry.xml).
  //
  // MENOR que kCaptureDistanceMm de propósito (não maior — primeira tentativa usou 250mm > 115mm
  // e o robô simplesmente parava boiando a ~135mm ATRÁS da bola, sem nunca tocá-la: a essa
  // distância o ponto é alcançável no espaço vazio, sem nenhum contato físico no caminho, então o
  // freio de chegada de movement_calculation.cpp — kArrivedDistanceMm=10mm — disparava
  // normalmente e travava o robô ali mesmo, "atrás da bola", exatamente o sintoma reportado).
  // kCaptureDistanceMm (115mm) já é a distância física real de contato (raio do robô + raio da
  // bola) — mirar em qualquer coisa MAIOR que isso é sempre alcançável sem tocar a bola.
  //
  // NEGATIVO (não só "menor que 115"): a velocidade em movement_calculation.cpp é proporcional à
  // distância residual até o alvo (speed = min(max_lin, dist/1000 * p_gain_linear), com
  // max_lin=1.0 m/s e p_gain_linear=2.0 por padrão) — com 30mm (residual sustentado ~85mm), a
  // velocidade nunca passava de ~0.17 m/s (17% do máximo), mesmo com o robô perfeitamente
  // alinhado. Precisa de mais distância residual pra sustentar mais velocidade pelo mesmo
  // controlador proporcional, e isso significa mirar além do CENTRO da bola, do lado do gol
  // (valor negativo) — o contato físico ainda segura o robô nos mesmos ~115mm reais, então a
  // distância residual sustentada vira 115mm - (-120mm) = 235mm, dando ~0.47 m/s (quase metade do
  // máximo). Continua na mesma linha bola→gol que o ponto de captura, recalculada a cada tick a
  // partir da posição atual da bola, então desvio lateral já é corrigido no tick seguinte (mesmo
  // mecanismo que kCaptureDistanceMm já usa).
  static constexpr double kCarryDistanceMm = -120.0;

  // Valor DEFAULT da porta "carry_face_lookahead_mm" (ver providedPorts()) — mesmo raciocínio de
  // kCarryDistanceMm acima. Distância (mm) do "ponto de mira da carga" (intercept_carry_face_x/y)
  // até a bola, do lado do GOL (não do lado do robô, como carry_distance_mm). Usado como
  // face_x/face_y do GoToPoint na árvore de carregar: mirar direto no gol (fixo, ~2-4m de
  // distância) fazia o robô zigueaguear —
  // qualquer desvio lateral da bola muda o ângulo bola→gol, e como o alvo de posição
  // (intercept_carry_x/y) segue a bola local mas o alvo de orientação era um ponto fixo distante,
  // a correção de ângulo não acompanhava a mesma linha local, gerando sobrecorreção. Este ponto
  // fica um pouco À FRENTE da bola, na mesma linha bola→gol, recalculado a cada tick a partir da
  // posição atual da bola — acompanha a linha reta local de verdade (como intercept_carry_x/y já
  // faz do outro lado), mas está longe o suficiente da bola (não é a bola em si) pra não ter a
  // instabilidade de bearing de mirar em algo muito perto.
  static constexpr double kCarryFaceLookaheadMm = 400.0;

  // Deslocamento lateral (mm) do waypoint de contorno (intercept_pk_safe_x/y) em relação à bola,
  // usado só enquanto o robô está do lado ERRADO dela (mesmo lado do gol). Ver comentário grande
  // em computeSafePreKick() no .cpp: o D* só evita colidir com a bola como obstáculo, não tem
  // noção de "por qual lado contornar" — se o caminho mais curto até intercept_pk_x/y (atrás da
  // bola) passa perto da bola pela frente, o D* vai por ali mesmo, deixando o robô do lado errado
  // (confirmado em teste: reposicionar a bola perto/atrás do robô faz ele ir pra frente dela
  // repetidamente, mesmo com ball_side_check forçando replanejamento — o replanejamento usa o
  // mesmo D* sem noção de lado, então comete o mesmo erro de novo). Precisa ser grande o
  // suficiente pra que o caminho até o waypoint não precise passar perto da bola.
  static constexpr double kSwingLateralOffsetMm = 600.0;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void ballPredictionCallback(const oxebots_interfaces::msg::BallPrediction::SharedPtr msg);
  void robotPredictionCallback(const oxebots_interfaces::msg::RobotPrediction::SharedPtr msg);
  void publishMarkers(double ball_x, double ball_y, double pk_x, double pk_y, double rx, double ry, bool is_ready);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<oxebots_interfaces::msg::BallPrediction>::SharedPtr ball_pred_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::RobotPrediction>::SharedPtr robot_pred_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  oxebots_interfaces::msg::BallPrediction::SharedPtr last_ball_pred_;
  oxebots_interfaces::msg::RobotPrediction::SharedPtr last_robot_pred_;
  std::mutex data_mutex_;

  // Histerese do waypoint de contorno (ver kSwingLateralOffsetMm): entra assim que o robô é
  // detectado do lado errado, só desarma quando ele já está seguramente do lado certo (não no
  // instante exato em que cruza o eixo bola-gol) — sem isso, perto do limiar o alvo publicado
  // oscilaria entre o waypoint e o pré-chute direto a cada tick.
  bool swing_active_ = false;
};

} // namespace oxebots_strategy
