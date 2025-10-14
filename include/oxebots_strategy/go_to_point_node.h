#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include <optional>

namespace oxebots_strategy
{

class GoToPointNode : public BT::StatefulActionNode
{
public:
  GoToPointNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
  std::optional<oxebots_interfaces::msg::RobotGameData> getRobotData(unsigned int robot_id);

  // Nós ROS e Comunicação
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;

  // Estado do Mundo e do Robô
  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  unsigned int robot_id_;
  geometry_msgs::msg::Point target_pos_;
};

} // namespace oxebots_strategy