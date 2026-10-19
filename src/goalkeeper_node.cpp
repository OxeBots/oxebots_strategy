#include "oxebots_strategy/goalkeeper_node.h"
#include <cmath>
#include <algorithm> 

namespace {
double normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
} // namespace

namespace oxebots_strategy
{

GoalkeeperNode::GoalkeeperNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr)
  : BT::StatefulActionNode(name, config), node_(node_ptr), ball_pos_updated_(false), robot_data_updated_(false)
{
  // Obter robot_id
  if (!getInput<uint32_t>("robot_id", robot_id_)) {
      robot_id_ = 0;
  }
  if (!getInput<double>("kick_speed", kick_speed_)) {
      kick_speed_ = 3.0;
  }

  // Obter my_goal_x e is_yellow do blackboard
  auto blackboard = config.blackboard;
  if (blackboard) {
      if (!blackboard->get("my_goal_x", my_goal_x_)) {
          my_goal_x_ = -2200.0;
      }
      if (!blackboard->get("is_yellow", is_yellow_team_)) {
          is_yellow_team_ = false;
      }
  }

  // Publisher para comandos de objetivo do robô (/robot_goal)
  auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);

  // Publisher para override de chute (/robot_motion_override)
  auto override_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  override_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotMotionOverride>("/robot_motion_override", override_qos);

  // Subscriber para os dados do jogo (/game_data contém bola e posições dos robôs)
  game_data_subscription_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10, std::bind(&GoalkeeperNode::game_data_callback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
              "GoalkeeperNode configurado para o robô %u. Meu gol em X: %.1f. Área fixa: 500mm em X e 1350mm em Y.",
              robot_id_, my_goal_x_);
}

BT::PortsList GoalkeeperNode::providedPorts()
{
  return {
    BT::InputPort<uint32_t>("robot_id", 0, "ID do robô goleiro"),
    BT::InputPort<double>("kick_speed", 3.0, "Velocidade do chute em m/s")
  };
}

void GoalkeeperNode::game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  last_ball_pos_ = msg->ball;
  ball_pos_updated_ = true;

  for (const auto& ally : msg->robots.allies) {
    if (ally.id == robot_id_) {
      current_robot_data_ = ally;
      robot_data_updated_ = true;
      return;
    }
  }
}

void GoalkeeperNode::clampInsidePenaltyArea(double & tx, double & ty, double margin_mm)
{
  // Dimensões fixas: 1350mm em Y (half_width = 675mm), 500mm em X
  double half_width = PENALTY_AREA_WIDTH / 2.0;
  if (my_goal_x_ > 0) {
    // Time amarelo (defendendo lado direito positivo)
    double min_x = my_goal_x_ - PENALTY_AREA_DEPTH - margin_mm;
    double max_x = my_goal_x_;
    tx = std::clamp(tx, min_x, max_x);
  } else {
    // Time azul (defendendo lado esquerdo negativo)
    double min_x = my_goal_x_;
    double max_x = my_goal_x_ + PENALTY_AREA_DEPTH + margin_mm;
    tx = std::clamp(tx, min_x, max_x);
  }
  double max_y = half_width + margin_mm;
  ty = std::clamp(ty, -max_y, max_y);
}

bool GoalkeeperNode::isBallInArea(double ball_x, double ball_y, double margin_mm)
{
  double half_width = PENALTY_AREA_WIDTH / 2.0;
  double max_y = half_width + margin_mm;
  if (std::abs(ball_y) > max_y) return false;

  if (my_goal_x_ > 0) {
    double min_x = my_goal_x_ - PENALTY_AREA_DEPTH - margin_mm;
    double max_x = my_goal_x_;
    return (ball_x >= min_x && ball_x <= max_x);
  } else {
    double min_x = my_goal_x_;
    double max_x = my_goal_x_ + PENALTY_AREA_DEPTH + margin_mm;
    return (ball_x >= min_x && ball_x <= max_x);
  }
}

void GoalkeeperNode::publishGoal(double x, double y, double target_w, uint8_t planner_type)
{
  bool pos_changed = (std::abs(x - last_target_x_) > 5.0 || std::abs(y - last_target_y_) > 5.0);
  bool ang_changed = (std::abs(normalizeAngle(target_w - last_target_w_)) > 0.05);
  bool planner_changed = (planner_type != last_planner_type_);

  if (pos_changed || ang_changed || planner_changed) {
    auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
    goal_msg->robot_id = robot_id_;
    goal_msg->pose.header.stamp = node_->now();
    goal_msg->pose.header.frame_id = "map";
    goal_msg->planner_type = planner_type;
    goal_msg->pose.pose.position.x = x / 1000.0;
    goal_msg->pose.pose.position.y = y / 1000.0;
    goal_msg->pose.pose.orientation.z = std::sin(target_w * 0.5);
    goal_msg->pose.pose.orientation.w = std::cos(target_w * 0.5);

    goal_pub_->publish(std::move(goal_msg));

    last_target_x_ = x;
    last_target_y_ = y;
    last_target_w_ = target_w;
    last_planner_type_ = planner_type;
  }
}

void GoalkeeperNode::sendKickOverride(double kick_speed)
{
  auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
  msg->robot_id = robot_id_;
  msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK;
  msg->kick_speed = std::min(kick_speed, 3.0);
  override_pub_->publish(std::move(msg));
  kick_override_active_ = true;
}

void GoalkeeperNode::releaseKickOverride()
{
  if (kick_override_active_) {
    auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
    msg->robot_id = robot_id_;
    msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_NONE;
    override_pub_->publish(std::move(msg));
    kick_override_active_ = false;
  }
}

BT::NodeStatus GoalkeeperNode::onStart()
{
  if (robot_id_ != 0) {
    RCLCPP_WARN_ONCE(node_->get_logger(), "GoalkeeperNode atribuído ao robô %u (esperado robô 0)", robot_id_);
  }

  (void)getInput<double>("kick_speed", kick_speed_);

  last_target_x_ = -99999.0;
  last_target_y_ = -99999.0;
  last_target_w_ = -99999.0;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoalkeeperNode::onRunning()
{
  if (!ball_pos_updated_ || !robot_data_updated_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "GoalkeeperNode: Aguardando dados de bola/robô...");
    return BT::NodeStatus::RUNNING;
  }

  double ball_x = last_ball_pos_.x;
  double ball_y = last_ball_pos_.y;
  double rx = current_robot_data_.x;
  double ry = current_robot_data_.y;
  double ryaw = current_robot_data_.orientation;

  // Sem margem de erro: estritamente dentro da área (500mm em X, 1350mm em Y)
  constexpr double kGkMarginMm = 0.0;

  bool ball_in_area = isBallInArea(ball_x, ball_y, kGkMarginMm);

  if (ball_in_area) {
    // --- ESTADO 1: Bola na área (ou na margem de 30cm) -> Sair, ir até a bola e chutar para as laterais ---
    // Alvo lateral para o chute (fora da área, de preferência para os lados)
    double target_side_y = (ball_y >= 0.0) ? 2000.0 : -2000.0;
    double target_side_x = 0.0; // Meio de campo

    double clear_angle = std::atan2(target_side_y - ball_y, target_side_x - ball_x);
    double ux = std::cos(clear_angle);
    double uy = std::sin(clear_angle);

    double dx = ball_x - rx;
    double dy = ball_y - ry;
    double dist_to_ball = std::hypot(dx, dy);

    // Produto escalar entre vetor robô->bola e a direção do chute
    // dot > 0 significa que a bola está à frente do robô na direção do chute (robô atrás da bola)
    double dot = (dx * ux + dy * uy);
    double angle_err = normalizeAngle(clear_angle - ryaw);

    if (dot > 0.0 && dist_to_ball < 250.0) {
      // Já posicionado atrás da bola: avança em linha reta para contato
      double target_x = ball_x + ux * 60.0;
      double target_y = ball_y + uy * 60.0;
      clampInsidePenaltyArea(target_x, target_y, kGkMarginMm);

      publishGoal(target_x, target_y, clear_angle, oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE);

      // Perto da bola e alinhado no ângulo: chutar!
      if (dist_to_ball < 180.0 && std::abs(angle_err) < 0.45) {
        sendKickOverride(kick_speed_);
      } else {
        releaseKickOverride();
      }
    } else {
      // Robô ainda longe ou fora da linha do chute: navegar até o ponto de pré-chute atrás da bola
      releaseKickOverride();
      double target_x = ball_x - ux * 200.0;
      double target_y = ball_y - uy * 200.0;
      clampInsidePenaltyArea(target_x, target_y, kGkMarginMm);

      publishGoal(target_x, target_y, clear_angle, oxebots_interfaces::msg::RobotGoal::PLANNER_DSTAR);
    }
  } else {
    // --- ESTADO 2: Bola fora da área -> Defende na linha do gol acompanhando Y ---
    releaseKickOverride();

    double offset_x = (my_goal_x_ > 0) ? -120.0 : 120.0;
    double target_x = my_goal_x_ + offset_x;

    double limit_y = (GOAL_WIDTH / 2.0) - 50.0;
    double target_y = std::clamp(ball_y, -limit_y, limit_y);

    double target_w = std::atan2(ball_y - target_y, ball_x - target_x);

    clampInsidePenaltyArea(target_x, target_y, kGkMarginMm);
    publishGoal(target_x, target_y, target_w, oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE);
  }

  return BT::NodeStatus::RUNNING;
}

void GoalkeeperNode::onHalted()
{
  releaseKickOverride();
  RCLCPP_INFO(node_->get_logger(), "GoalkeeperNode: Robô %u parado.", robot_id_);
}

}  // namespace oxebots_strategy
