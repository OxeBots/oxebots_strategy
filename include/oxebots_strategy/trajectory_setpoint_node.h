#pragma once

#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/robot_prediction.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/robot_trajectory_setpoint.hpp"

#include <optional>

// Projeta a última velocidade instantânea decidida por movement_calculation_node
// (/robot_commands) num setpoint de trajetória (ponto futuro + velocidade nesse ponto + ângulo),
// mais a pose atual do Kalman como referência de correção — para o modo "point" do rádio NRF24
// (ver Nrf24.cpp). Não recalcula path, perfil de velocidade, chute nem alinhamento: tudo isso
// continua sendo decidido uma única vez em movement_calculation_node; este nó só traduz a
// decisão já tomada para um formato diferente. O grSim e o modo "velocity" do rádio continuam
// usando /robot_commands diretamente, sem nenhuma mudança.
class TrajectorySetpointNode : public rclcpp::Node
{
public:
  TrajectorySetpointNode();

private:
  void robotCmdCallback(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg);
  void robotPredictionCallback(const oxebots_interfaces::msg::RobotPrediction::SharedPtr msg);
  void robotGoalCallback(const oxebots_interfaces::msg::RobotGoal::SharedPtr msg);

  rclcpp::Subscription<oxebots_interfaces::msg::RobotCmd>::SharedPtr robot_cmd_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::RobotPrediction>::SharedPtr robot_prediction_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::RobotGoal>::SharedPtr robot_goal_sub_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotTrajectorySetpoint>::SharedPtr setpoint_pub_;

  int robot_id_ = 0;
  double radio_publish_rate_hz_ = 15.0;
  double lookahead_safety_factor_ = 1.5;
  double min_lookahead_mm_ = 50.0;
  double max_lookahead_mm_ = 1000.0;

  // Última pose/velocidade do Kalman para este robô — base da projeção e também o que vira
  // "vision" no setpoint publicado. Sem isso ainda, não há base pra projetar, então não publica.
  std::optional<oxebots_interfaces::msg::RobotPredictionData> last_prediction_;

  // Ângulo alvo (de /robot_goal), decodificado do quatérnio — mesma fórmula usada em
  // movement_calculation.cpp e go_to_point_node.cpp, pra ficar consistente com o resto do
  // sistema. Sem isso ainda, usa a orientação atual do Kalman como alvo (fica parado olhando
  // pra onde já está, até o primeiro /robot_goal chegar).
  std::optional<double> target_angle_;
};
