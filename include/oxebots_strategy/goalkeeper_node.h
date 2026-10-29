#ifndef OXEBOTS_STRATEGY__GOALKEEPER_NODE_H_
#define OXEBOTS_STRATEGY__GOALKEEPER_NODE_H_

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/robot_motion_override.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/ball_position.hpp"

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
  void game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg);

  void clampInsidePenaltyArea(double & tx, double & ty, double margin_mm);
  bool isBallInArea(double ball_x, double ball_y, double margin_mm);
  void publishGoal(double x, double y, double target_w, uint8_t planner_type);
  void sendKickOverride(double kick_speed);
  void releaseKickOverride();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotMotionOverride>::SharedPtr override_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_subscription_;

  uint32_t robot_id_{0};
  double my_goal_x_{0.0};
  bool is_yellow_team_{false};
  double kick_speed_{3.0};

  // Dimensões do campo e da área lidas dinamicamente do Blackboard (originadas de /field_geometry)
  double penalty_area_depth_{0.0};
  double penalty_area_width_{0.0};
  double goal_width_{0.0};
  double field_width_{0.0};

  oxebots_interfaces::msg::BallPosition last_ball_pos_;
  oxebots_interfaces::msg::RobotGameData current_robot_data_;
  bool ball_pos_updated_{false};
  bool robot_data_updated_{false};

  double last_target_x_{-99999.0};
  double last_target_y_{-99999.0};
  double last_target_w_{-99999.0};
  uint8_t last_planner_type_{oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE};

  bool kick_override_active_{false};
};

}  // namespace oxebots_strategy

#endif  // OXEBOTS_STRATEGY__GOALKEEPER_NODE_H_
