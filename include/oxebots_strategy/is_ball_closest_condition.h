#ifndef OXEBOTS_STRATEGY__IS_BALL_CLOSEST_CONDITION_H_
#define OXEBOTS_STRATEGY__IS_BALL_CLOSEST_CONDITION_H_

#include <mutex>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/ball_prediction.hpp"

namespace oxebots_strategy
{

// SUCCESS quando a bola está mais perto do robô (goleiro) do que de qualquer outro robô, aliado ou
// adversário, perto o bastante do nosso gol e sem ser um chute a caminho do gol (esse caso é da interceptação).
class IsBallClosestCondition : public BT::ConditionNode
{
public:
  IsBallClosestCondition(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
  void ballPredictionCallback(const oxebots_interfaces::msg::BallPrediction::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::BallPrediction>::SharedPtr ball_pred_sub_;

  std::mutex data_mutex_;
  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  oxebots_interfaces::msg::BallPrediction::SharedPtr last_ball_pred_;
  rclcpp::Time last_ball_pred_stamp_{0, 0, RCL_ROS_TIME};

  bool last_status_ = false;
};

} // namespace oxebots_strategy

#endif // OXEBOTS_STRATEGY__IS_BALL_CLOSEST_CONDITION_H_
