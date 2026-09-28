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

  // Default da porta "carry_distance_mm" (ver providedPorts()): distância (mm) do "ponto de
  // carga" (intercept_carry_x/y) até a bola, do lado do GOL — negativo, então o ponto fica além
  // do centro da bola. O contato físico (~115mm, kCaptureDistanceMm) segura o robô bem antes de
  // lá, mas como a velocidade em movement_calculation.cpp é proporcional à distância residual até
  // o alvo, mirar além sustenta mais velocidade de empurrão pelo mesmo controlador proporcional.
  static constexpr double kCarryDistanceMm = -120.0;

  // Default da porta "carry_face_lookahead_mm": distância (mm) do "ponto de mira da carga"
  // (intercept_carry_face_x/y) até a bola, do lado do gol — usado como face_x/face_y do GoToPoint
  // ao carregar. Fica na mesma linha bola→gol que intercept_carry_x/y, recalculado a cada tick a
  // partir da bola atual (evita zigue-zague de um alvo de orientação fixo/distante), mas longe o
  // bastante da bola pra não ter instabilidade de bearing.
  static constexpr double kCarryFaceLookaheadMm = 400.0;

  // Deslocamento lateral (mm) do waypoint de contorno (intercept_pk_safe_x/y), usado só enquanto
  // o robô está do lado errado da bola. Ver cálculo de intercept_pk_safe no .cpp.
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

  // Histerese do waypoint de contorno: entra ao detectar lado errado, só desarma com folga real.
  bool swing_active_ = false;
};

} // namespace oxebots_strategy
