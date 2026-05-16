#ifndef OXEBOTS_STRATEGY__IS_BALL_IN_AREA_CONDITION_H_
#define OXEBOTS_STRATEGY__IS_BALL_IN_AREA_CONDITION_H_

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

namespace oxebots_strategy
{

class IsBallInAreaCondition : public BT::ConditionNode
{
public:
  IsBallInAreaCondition(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  bool last_status_ = false;
};

} // namespace oxebots_strategy

#endif // OXEBOTS_STRATEGY__IS_BALL_IN_AREA_CONDITION_H_
