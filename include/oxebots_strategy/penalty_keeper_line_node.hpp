#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "rclcpp/rclcpp.hpp"

namespace oxebots_strategy
{

// Folha de pênalti defensivo (Capítulo 18, Capítulo 21, K02, R-15, SC-10):
// Leva o goleiro à linha de gol (a `line_offset_mm` = 90 mm dela para dentro do campo, tocando
// a linha com o raio do chassi) e o mantém lá acompanhando o Y da bola entre as traves, sem
// sair da linha mesmo que a bola entre na área, devolvendo RUNNING enquanto ativo e FAILURE
// se faltarem dados ou geometria.
class PenaltyKeeperLineNode : public BT::StatefulActionNode
{
public:
  PenaltyKeeperLineNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
  BT::NodeStatus step();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;

  std::mutex mutex_;
  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  rclcpp::Time last_game_data_stamp_{0, 0, RCL_ROS_TIME};

  uint32_t robot_id_{0};
  double default_line_offset_mm_{90.0};

  double last_target_x_{-99999.0};
  double last_target_y_{-99999.0};
  double last_target_w_{-99999.0};
  std::optional<rclcpp::Time> last_publish_time_;
};

}  // namespace oxebots_strategy
