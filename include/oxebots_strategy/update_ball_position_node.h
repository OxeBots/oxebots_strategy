#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"

namespace oxebots_strategy
{

// Este nó subscreve ao /game_data e escreve a posição da bola no blackboard
class UpdateBallPositionNode : public BT::SyncActionNode
{
public:
  UpdateBallPositionNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts() {
    return { BT::OutputPort<double>("ball_x"), BT::OutputPort<double>("ball_y") };
  }

  BT::NodeStatus tick() override;

private:
  void gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  std::mutex data_mutex_;
};

} // namespace oxebots_strategy
