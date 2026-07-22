#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"

namespace oxebots_strategy
{

// Este nó subscreve ao /game_data e escreve a posição da bola no blackboard
class UpdateBallPositionNode : public BT::StatefulActionNode
{
public:
  UpdateBallPositionNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  // Distância (mm) do ponto de captura até o centro da bola: perto o bastante para o dribbler
  // encostar na bola já alinhado, mas sem colidir de forma descontrolada.
  static constexpr double kCaptureDistanceMm = 90.0;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  std::mutex data_mutex_;
};

} // namespace oxebots_strategy
