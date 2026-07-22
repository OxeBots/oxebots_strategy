#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_motion_override.hpp"
#include "oxebots_interfaces/msg/robot_motion_status.hpp"

namespace oxebots_strategy
{

// Pede ao controlador central de movimento (movement_calculation_node) que gire o robô no
// lugar até a frente dele apontar para a bola REAL (não um alvo idealizado calculado antes de
// chegar), via /robot_motion_override. É o controlador quem lê a bola ao vivo e faz o controle
// angular; este nó só publica o pedido e espera "aligned" chegar em /robot_motion_status — ele
// não escreve mais em /robot_commands diretamente (isso é o que causava a corrida com o
// PathFollower/KickBall descrita no histórico do git).
class AlignToBallNode : public BT::StatefulActionNode
{
public:
  AlignToBallNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus step();
  void statusCallback(const oxebots_interfaces::msg::RobotMotionStatus::SharedPtr msg);
  void releaseOverride();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotMotionOverride>::SharedPtr override_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::RobotMotionStatus>::SharedPtr status_sub_;
  oxebots_interfaces::msg::RobotMotionStatus::SharedPtr last_status_;

  unsigned int robot_id_ = 0;
};

} // namespace oxebots_strategy
