#include "oxebots_strategy/trajectory_setpoint_node.h"

#include <algorithm>
#include <cmath>

TrajectorySetpointNode::TrajectorySetpointNode()
: rclcpp::Node("trajectory_setpoint_node")
{
  robot_id_ = this->declare_parameter("robot_id", robot_id_);
  radio_publish_rate_hz_ = this->declare_parameter("radio_publish_rate_hz", radio_publish_rate_hz_);
  lookahead_safety_factor_ = this->declare_parameter("lookahead_safety_factor", lookahead_safety_factor_);
  min_lookahead_mm_ = this->declare_parameter("min_lookahead_mm", min_lookahead_mm_);
  max_lookahead_mm_ = this->declare_parameter("max_lookahead_mm", max_lookahead_mm_);

  robot_prediction_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotPrediction>(
    "robot_predicted", rclcpp::SensorDataQoS(),
    std::bind(&TrajectorySetpointNode::robotPredictionCallback, this, std::placeholders::_1));

  robot_goal_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotGoal>(
    "/robot_goal", rclcpp::SensorDataQoS(),
    std::bind(&TrajectorySetpointNode::robotGoalCallback, this, std::placeholders::_1));

  robot_cmd_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotCmd>(
    "/robot_commands", rclcpp::SensorDataQoS(),
    std::bind(&TrajectorySetpointNode::robotCmdCallback, this, std::placeholders::_1));

  setpoint_pub_ = this->create_publisher<oxebots_interfaces::msg::RobotTrajectorySetpoint>(
    "/robot_trajectory_setpoints", rclcpp::SensorDataQoS());
}

void TrajectorySetpointNode::robotPredictionCallback(
  const oxebots_interfaces::msg::RobotPrediction::SharedPtr msg)
{
  for (const auto & ally : msg->allies) {
    if (static_cast<int>(ally.id) == robot_id_) {
      last_prediction_ = ally;
      return;
    }
  }
}

void TrajectorySetpointNode::robotGoalCallback(const oxebots_interfaces::msg::RobotGoal::SharedPtr msg)
{
  if (msg->robot_id != robot_id_) {
    return;
  }
  const auto & q = msg->pose.pose.orientation;
  target_angle_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void TrajectorySetpointNode::robotCmdCallback(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg)
{
  if (!last_prediction_.has_value()) {
    return;
  }

  for (const auto & robot_cmd : msg->robots) {
    if (static_cast<int>(robot_cmd.id) != robot_id_) {
      continue;
    }

    const auto & kalman = last_prediction_.value();

    // dt cobre o intervalo até o próximo setpoint realmente chegar no robô (período de
    // publicação do rádio), mais uma margem de segurança — o robô roda em malha fechada nesse
    // ponto futuro enquanto nenhum setpoint novo chega.
    const double dt = (1.0 / radio_publish_rate_hz_) * lookahead_safety_factor_;

    const double x_v_mm_s = robot_cmd.x_velocity * 1000.0;
    const double y_v_mm_s = robot_cmd.y_velocity * 1000.0;
    const double speed_mm_s = std::hypot(x_v_mm_s, y_v_mm_s);

    double lookahead_mm = std::clamp(speed_mm_s * dt, min_lookahead_mm_, max_lookahead_mm_);
    double dir_x = 0.0, dir_y = 0.0;
    if (speed_mm_s > 1e-6) {
      dir_x = x_v_mm_s / speed_mm_s;
      dir_y = y_v_mm_s / speed_mm_s;
    }

    oxebots_interfaces::msg::RobotTrajectorySetpointData setpoint_data;
    setpoint_data.id = robot_cmd.id;

    setpoint_data.target.x = kalman.x + dir_x * lookahead_mm;
    setpoint_data.target.y = kalman.y + dir_y * lookahead_mm;
    setpoint_data.target.angle = target_angle_.value_or(kalman.orientation);
    setpoint_data.target.x_v = x_v_mm_s;
    setpoint_data.target.y_v = y_v_mm_s;
    setpoint_data.target.angular_vel = robot_cmd.angular_velocity;

    setpoint_data.vision.x = kalman.x;
    setpoint_data.vision.y = kalman.y;
    setpoint_data.vision.angle = kalman.orientation;
    setpoint_data.vision.x_v = kalman.vx;
    setpoint_data.vision.y_v = kalman.vy;
    setpoint_data.vision.angular_vel = kalman.vorientation;

    setpoint_data.kick_speed = robot_cmd.kick_speed;

    oxebots_interfaces::msg::RobotTrajectorySetpoint setpoint_msg;
    setpoint_msg.robots.push_back(setpoint_data);
    setpoint_pub_->publish(setpoint_msg);
    return;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectorySetpointNode>());
  rclcpp::shutdown();
  return 0;
}
