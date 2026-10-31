#pragma once

#include <mutex>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_strategy/goalkeeper_blackboard.hpp"

namespace oxebots_strategy
{

class LateralClearNode : public BT::StatefulActionNode
{
public:
  LateralClearNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // Decisão do chute: direção (vetor unitário), velocidade e, se for passe, o id do aliado
  struct KickPlan
  {
    double dir_x;
    double dir_y;
    double speed;
    int pass_id;   // -1 = afastar para a frente
  };

  void gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
  KickPlan planKick(double ball_x, double ball_y, const goalkeeper::FieldGeometry & geo, bool urgent);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;

  std::mutex data_mutex_;
  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;

  uint32_t robot_id_;
  double last_target_x_ = -99999.0;
  double last_target_y_ = -99999.0;
  double last_target_w_ = -99999.0;

  // Histerese da decisão de chute
  int last_pass_id_ = -1;
  bool opponent_close_ = false;
  bool plan_locked_ = false;
  KickPlan locked_plan_{1.0, 0.0, 3.0, -1};

  // Início do contato com a bola, para o tempo limite do alinhamento
  bool contact_since_valid_ = false;
  rclcpp::Time contact_since_{0, 0, RCL_ROS_TIME};

  // Início da jogada (a bola não pode ficar mais de 10 s na área de defesa)
  rclcpp::Time engage_start_{0, 0, RCL_ROS_TIME};
};

}
