#include "oxebots_strategy/trajectory_setpoint_node.h"

#include <algorithm>
#include <cmath>

TrajectorySetpointNode::TrajectorySetpointNode()
: rclcpp::Node("trajectory_setpoint_node")
{
  robot_id_ = this->declare_parameter("robot_id", robot_id_);
  radio_publish_rate_hz_ = this->declare_parameter("radio_publish_rate_hz", radio_publish_rate_hz_);
  lookahead_safety_factor_ = this->declare_parameter("lookahead_safety_factor", lookahead_safety_factor_);
  max_lookahead_mm_ = this->declare_parameter("max_lookahead_mm", max_lookahead_mm_);

  robot_prediction_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotPrediction>(
    "robot_predicted", rclcpp::SensorDataQoS(),
    std::bind(&TrajectorySetpointNode::robotPredictionCallback, this, std::placeholders::_1));

  // transient_local + reliable, igual ao publisher (go_to_point_node.cpp/defender_nodes.cpp):
  // sem isso, esse nó não recebe o último /robot_goal retido se subir depois dele ter sido
  // publicado (QoS volatile não repete histórico de late-joiner mesmo quando "compatível").
  robot_goal_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotGoal>(
    "/robot_goal", rclcpp::QoS(1).transient_local().reliable(),
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

    // Sem piso mínimo: com um piso, velocidade residual (ex.: 10mm/s na aproximação fina) ainda
    // força o alvo a um salto de min_lookahead_mm à frente, e quando a velocidade zera de vez o
    // alvo salta de volta pra posição atual — oscilação (hunting) bem na hora de alinhar fino.
    // Deixa decair continuamente a zero; só o teto (max_lookahead_mm) continua limitando.
    double lookahead_mm = std::min(speed_mm_s * dt, max_lookahead_mm_);
    double dir_x = 0.0, dir_y = 0.0;
    if (speed_mm_s > 1e-6) {
      dir_x = x_v_mm_s / speed_mm_s;
      dir_y = y_v_mm_s / speed_mm_s;
    }

    oxebots_interfaces::msg::RobotTrajectorySetpointData setpoint_data;
    setpoint_data.id = robot_cmd.id;

    setpoint_data.target.x = kalman.x + dir_x * lookahead_mm;
    setpoint_data.target.y = kalman.y + dir_y * lookahead_mm;
    // CAVEAT CONHECIDO: target.angle vem só de /robot_goal, que não é republicado em todo
    // estado (ex.: MODE_ALIGN_TO_BALL mira na bola em tempo real sem tocar /robot_goal;
    // após "arrived" o movement_calculation_node solta a autoridade angular via
    // target_w_.reset() sem republicar nada). Nesses estados target.angle fica desatualizado
    // enquanto target.angular_vel (abaixo) reflete o que o movement_calculation_node decidiu
    // de fato agora. O firmware precisa tratar angular_vel como autoritativo/feedforward
    // nesses casos, não como um valor que deveria convergir com target.angle a qualquer custo.
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
