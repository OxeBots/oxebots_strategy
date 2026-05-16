#include "oxebots_strategy/is_ball_in_our_half_condition.h"
#include <cmath>

namespace oxebots_strategy
{

static constexpr double HALF_FIELD_HYSTERESIS = 100.0;

IsBallInOurHalfCondition::IsBallInOurHalfCondition(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node) {}

BT::PortsList IsBallInOurHalfCondition::providedPorts()
{
  return { BT::InputPort<double>("ball_x") };
}

BT::NodeStatus IsBallInOurHalfCondition::tick()
{
  double ball_x, my_goal_x;

  if (!getInput<double>("ball_x", ball_x)) return BT::NodeStatus::FAILURE;
  
  auto blackboard = config().blackboard;
  if (!blackboard->get("my_goal_x", my_goal_x)) return BT::NodeStatus::FAILURE;

  bool in_our_half = last_status_;

  if (my_goal_x < 0) {
      if (ball_x <= 0.0) {
          in_our_half = true;
      } else if (ball_x > HALF_FIELD_HYSTERESIS) {
          in_our_half = false;
      }
  } else {
      if (ball_x >= 0.0) {
          in_our_half = true;
      } else if (ball_x < -HALF_FIELD_HYSTERESIS) {
          in_our_half = false;
      }
  }

  last_status_ = in_our_half;
  return in_our_half ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}
