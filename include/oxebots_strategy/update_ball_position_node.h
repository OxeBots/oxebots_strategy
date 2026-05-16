#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace oxebots_strategy
{

// Este nó subscreve ao /game_data e escreve a posição da bola no blackboard
class UpdateBallPositionNode : public BT::StatefulActionNode
{
public:
  UpdateBallPositionNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  std::mutex data_mutex_;
};

} // namespace oxebots_strategy
