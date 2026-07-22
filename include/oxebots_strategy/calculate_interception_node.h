#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/ball_prediction.hpp"
#include "oxebots_interfaces/msg/robot_prediction.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <mutex>
#include <optional>

namespace oxebots_strategy
{

class CalculateInterceptionNode : public BT::StatefulActionNode
{
public:
  CalculateInterceptionNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  // Distância (mm) do ponto de captura até a bola prevista: perto o bastante para o dribbler
  // encostar já alinhado, sem mirar no centro exato da bola (evita colisão de frente).
  static constexpr double kCaptureDistanceMm = 90.0;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void ballPredictionCallback(const oxebots_interfaces::msg::BallPrediction::SharedPtr msg);
  void robotPredictionCallback(const oxebots_interfaces::msg::RobotPrediction::SharedPtr msg);
  void publishMarkers(double ball_x, double ball_y, double pk_x, double pk_y, double rx, double ry, bool is_ready);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<oxebots_interfaces::msg::BallPrediction>::SharedPtr ball_pred_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::RobotPrediction>::SharedPtr robot_pred_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  oxebots_interfaces::msg::BallPrediction::SharedPtr last_ball_pred_;
  oxebots_interfaces::msg::RobotPrediction::SharedPtr last_robot_pred_;
  std::mutex data_mutex_;
};

} // namespace oxebots_strategy
