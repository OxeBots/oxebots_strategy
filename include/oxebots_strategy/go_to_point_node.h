#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/ssl_geometry_data.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <optional>

namespace oxebots_strategy
{

class GoToPointNode : public BT::StatefulActionNode
{
public:
  GoToPointNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  // Converte a porta string "planner" (ex: "straight_line") no valor uint8 de
  // RobotGoal::planner_type. Qualquer valor não reconhecido cai em PLANNER_DSTAR.
  static uint8_t plannerFromString(const std::string& planner);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void publishGoal();
  void gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
  void geometryDataCallback(const oxebots_interfaces::msg::SSLGeometryData::SharedPtr msg);
  void publishMarkers();
  std::optional<oxebots_interfaces::msg::RobotGameData> getRobotData(unsigned int robot_id);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::SSLGeometryData>::SharedPtr geometry_sub_;

  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  std::optional<oxebots_interfaces::msg::SSLFieldSize> field_size_;
  unsigned int robot_id_;
  geometry_msgs::msg::Point target_pos_;
  double target_w_;
};

} // namespace oxebots_strategy
