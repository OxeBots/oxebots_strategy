#ifndef OXEBOTS_STRATEGY__GOALKEEPER_NODE_H_
#define OXEBOTS_STRATEGY__GOALKEEPER_NODE_H_

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/game_data.hpp" // Para obter a posição do próprio robô, se necessário

namespace oxebots_strategy
{

class GoalkeeperNode : public BT::StatefulActionNode
{
public:
  GoalkeeperNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void ball_callback(const oxebots_interfaces::msg::BallPosition::SharedPtr msg);
  void game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::BallPosition>::SharedPtr ball_subscription_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_subscription_;
  
  int robot_id_;
  double my_goal_x_;
  bool is_yellow_team_; // Para saber a cor do time
  
  oxebots_interfaces::msg::BallPosition last_ball_pos_;
  oxebots_interfaces::msg::RobotGameData current_robot_data_; // Para a posição do próprio goleiro
  bool ball_pos_updated_;
  bool robot_data_updated_;
};

}  // namespace oxebots_strategy

#endif  // OXEBOTS_STRATEGY__GOALKEEPER_NODE_H_
