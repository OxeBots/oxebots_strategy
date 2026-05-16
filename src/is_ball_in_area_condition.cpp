#include "oxebots_strategy/is_ball_in_area_condition.h"

namespace oxebots_strategy
{

static constexpr double AREA_EXIT_MARGIN = 200.0;
static constexpr double AREA_ENTRY_TOLERANCE = 100.0;
static constexpr double GOAL_LINE_TOLERANCE = 100.0;

IsBallInAreaCondition::IsBallInAreaCondition(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node) {}

BT::PortsList IsBallInAreaCondition::providedPorts()
{
  return { BT::InputPort<double>("ball_x"),
           BT::InputPort<double>("ball_y") };
}

BT::NodeStatus IsBallInAreaCondition::tick()
{
  double ball_x, ball_y, my_goal_x, p_depth, p_width;

  if (!getInput<double>("ball_x", ball_x) || !getInput<double>("ball_y", ball_y)) return BT::NodeStatus::FAILURE;
  
  auto blackboard = config().blackboard;
  if (!blackboard->get("my_goal_x", my_goal_x) || 
      !blackboard->get("penalty_area_depth", p_depth) ||
      !blackboard->get("penalty_area_width", p_width)) {
    return BT::NodeStatus::FAILURE;
  }

  double exit_margin = last_status_ ? AREA_EXIT_MARGIN : 0.0;

  bool in_depth = false;
  if (my_goal_x > 0) {
    in_depth = (ball_x > (my_goal_x - p_depth - AREA_ENTRY_TOLERANCE - exit_margin)) && (ball_x < (my_goal_x + GOAL_LINE_TOLERANCE));
  } else {
    in_depth = (ball_x < (my_goal_x + p_depth + AREA_ENTRY_TOLERANCE + exit_margin)) && (ball_x > (my_goal_x - GOAL_LINE_TOLERANCE));
  }

  bool in_width = std::abs(ball_y) < (p_width / 2.0 + AREA_ENTRY_TOLERANCE + exit_margin);

  if (in_depth && in_width) {
    last_status_ = true;
    return BT::NodeStatus::SUCCESS;
  }

  last_status_ = false;
  return BT::NodeStatus::FAILURE;
}

}
