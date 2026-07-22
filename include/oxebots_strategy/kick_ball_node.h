#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_motion_override.hpp"
#include "oxebots_interfaces/msg/robot_motion_status.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"

namespace oxebots_strategy
{

// Pede ao controlador central de movimento (movement_calculation_node) que execute o chute,
// via /robot_motion_override. O disparo em si (avanço + kick_speed, janela de contato,
// cooldown entre chutes) roda inteiramente no controlador; este nó só publica o pedido e
// espera o contador de chutes subir em /robot_motion_status, em vez de escrever direto em
// /robot_commands (o que causava corrida com o PathFollower — ver histórico do git).
class KickBallNode : public BT::StatefulActionNode
{
public:
  KickBallNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus step();
  void gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
  void statusCallback(const oxebots_interfaces::msg::RobotMotionStatus::SharedPtr msg);
  void logKickGeometry(unsigned int robot_id);
  void releaseOverride();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotMotionOverride>::SharedPtr override_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::RobotMotionStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  oxebots_interfaces::msg::RobotMotionStatus::SharedPtr last_status_;

  unsigned int robot_id_ = 0;
  double kick_speed_ = 0.0;

  // Registrado em onStart(), ANTES do primeiro pedido: SUCCESS só chega quando o controlador
  // reportar um contador maior que este, nunca por um booleano que seria ambíguo entre "ainda
  // não disparou" e "já disparou e voltou a ficar ocioso".
  uint32_t baseline_kick_count_ = 0;
};

} // namespace oxebots_strategy
