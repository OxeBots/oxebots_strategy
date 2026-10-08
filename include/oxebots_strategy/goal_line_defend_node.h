#ifndef OXEBOTS_STRATEGY__GOAL_LINE_DEFEND_NODE_H_
#define OXEBOTS_STRATEGY__GOAL_LINE_DEFEND_NODE_H_

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/ball_prediction.hpp"
#include <mutex>

namespace oxebots_strategy
{

class GoalLineDefendNode : public BT::StatefulActionNode
{
public:
  GoalLineDefendNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void ballPredictionCallback(const oxebots_interfaces::msg::BallPrediction::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::BallPrediction>::SharedPtr ball_pred_sub_;

  std::mutex data_mutex_;
  oxebots_interfaces::msg::BallPrediction::SharedPtr last_ball_pred_;

  uint32_t robot_id_;
  double my_goal_x_;
  double last_target_y_ = -99999.0;
};

} // namespace oxebots_strategy

#endif // OXEBOTS_STRATEGY__GOAL_LINE_DEFEND_NODE_H_
