#ifndef OXEBOTS_STRATEGY__GOALKEEPER_NODE_H_
#define OXEBOTS_STRATEGY__GOALKEEPER_NODE_H_

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_strategy/rrt_star_planner.hpp"
#include <vector>

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
  Point find_lookahead_point(const std::vector<Point>& path, const Point& robot_pos, double lookahead_distance);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_subscription_;
  
  int robot_id_;
  double my_goal_x_;
  bool is_yellow_team_;
  
  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  bool robot_data_updated_;
  
  std::vector<Point> current_path_;
  Point lookahead_point_;
  Point target_pos_;
  double target_w_;
};

}  // namespace oxebots_strategy

#endif  // OXEBOTS_STRATEGY__GOALKEEPER_NODE_H_
