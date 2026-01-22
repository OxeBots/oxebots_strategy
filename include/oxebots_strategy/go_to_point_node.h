#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_strategy/rrt_star_planner.hpp"
#include <optional>
#include <vector>

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
  Point find_lookahead_point(const std::vector<Point>& path, const Point& robot_pos, double lookahead_distance);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;

  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  unsigned int robot_id_;
  Point target_pos_;
  double target_w_;

  // RRT* related data
  std::vector<Point> current_path_;
  Point lookahead_point_;
};

} // namespace oxebots_strategy
