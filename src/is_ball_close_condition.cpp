#include "oxebots_strategy/is_ball_close_condition.h"
#include <cmath>

namespace oxebots_strategy
{

IsBallCloseCondition::IsBallCloseCondition(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node)
{
  game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10, std::bind(&IsBallCloseCondition::gameDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "IsBallCloseCondition configurado.");
}

void IsBallCloseCondition::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_game_data_ = msg;
  if (node_) {
    last_game_data_time_ = node_->now();
  }
}

BT::PortsList IsBallCloseCondition::providedPorts()
{
  return { BT::InputPort<unsigned int>("robot_id"),
           BT::InputPort<double>("distance_threshold", 180.0, "Distância para considerar 'perto'"),
           BT::InputPort<double>("angle_threshold_deg", -1.0,
             "Desvio angular máximo (graus) entre a frente do robô e a bola (ou alvo, se target_x/target_y forem fornecidos), para o kicker "
             "realmente alcançar a bola. Se <= 0, não checa ângulo (comportamento antigo)."),
           BT::InputPort<double>("target_x", "Opcional: coordenada X (mm) do alvo de chute para validar o alinhamento bola->alvo"),
           BT::InputPort<double>("target_y", "Opcional: coordenada Y (mm) do alvo de chute para validar o alinhamento bola->alvo") };
}

double IsBallCloseCondition::normalizeAngleDeg(double angle)
{
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

BT::NodeStatus IsBallCloseCondition::tick()
{
  unsigned int robot_id;
  double threshold;
  double angle_threshold_deg = -1.0;
  if (!getInput<unsigned int>("robot_id", robot_id) || !getInput<double>("distance_threshold", threshold)) {
    RCLCPP_ERROR(node_->get_logger(), "Faltando portas [robot_id] ou [distance_threshold]");
    return BT::NodeStatus::FAILURE;
  }
  getInput<double>("angle_threshold_deg", angle_threshold_deg);

  std::lock_guard<std::mutex> lock(data_mutex_);
  auto * bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;

  double max_pose_ms = 150.0;
  double max_ball_ms = 300.0;
  if (bb) {
    (void)bb->get("max_pose_ms", max_pose_ms);
    (void)bb->get("max_ball_ms", max_ball_ms);
    double bb_pose_age = 0.0;
    if (bb->get("pose_age_ms", bb_pose_age) && (!std::isfinite(bb_pose_age) || bb_pose_age > max_pose_ms)) {
      return BT::NodeStatus::FAILURE;
    }
    double bb_ball_age = 0.0;
    if (bb->get("ball_age_ms", bb_ball_age) && (!std::isfinite(bb_ball_age) || bb_ball_age > max_ball_ms)) {
      return BT::NodeStatus::FAILURE;
    }
  }

  double robot_x = 0.0, robot_y = 0.0, robot_orientation = 0.0;
  double ball_x = 0.0, ball_y = 0.0;
  bool robot_found = false;
  bool ball_found = false;

  if (last_game_data_) {
    const auto now_time = node_->now();
    const bool has_ball_stamp = (last_game_data_->ball.stamp.sec != 0 || last_game_data_->ball.stamp.nanosec != 0);
    if (has_ball_stamp && last_game_data_->ball.confidence <= 0.0f) {
      return BT::NodeStatus::FAILURE;
    }
    const rclcpp::Time ball_time =
      has_ball_stamp ? rclcpp::Time(last_game_data_->ball.stamp, now_time.get_clock_type()) : last_game_data_time_;
    const double ball_age_ms = (now_time - ball_time).nanoseconds() / 1e6;
    if (!std::isfinite(ball_age_ms) || ball_age_ms > max_ball_ms) {
      return BT::NodeStatus::FAILURE;
    }
    ball_x = last_game_data_->ball.x;
    ball_y = last_game_data_->ball.y;
    ball_found = true;

    for (const auto& ally : last_game_data_->robots.allies) {
      if (ally.id == robot_id) {
        const bool has_robot_stamp = (ally.stamp.sec != 0 || ally.stamp.nanosec != 0);
        const rclcpp::Time robot_time =
          has_robot_stamp ? rclcpp::Time(ally.stamp, now_time.get_clock_type()) : last_game_data_time_;
        const double pose_age_ms = (now_time - robot_time).nanoseconds() / 1e6;
        if (!std::isfinite(pose_age_ms) || pose_age_ms > max_pose_ms) {
          return BT::NodeStatus::FAILURE;
        }
        robot_x = ally.x;
        robot_y = ally.y;
        robot_orientation = ally.orientation;
        robot_found = true;
        break;
      }
    }
  } else if (bb) {
    uint32_t bb_robot_id = 0;
    if (bb->get("robot_id", bb_robot_id) && bb_robot_id == robot_id) {
      robot_found = bb->get("robot_x", robot_x) && bb->get("robot_y", robot_y);
      (void)bb->get("robot_yaw", robot_orientation);
    }
    ball_found = bb->get("ball_x", ball_x) && bb->get("ball_y", ball_y);
  }

  if (!ball_found) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Nenhum dado de jogo recebido.");
    return BT::NodeStatus::FAILURE;
  }

  if (!robot_found) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Robô com ID %d não encontrado.", robot_id);
    return BT::NodeStatus::FAILURE;
  }

  double dx = ball_x - robot_x;
  double dy = ball_y - robot_y;
  double distance = std::hypot(dx, dy);

  if (distance >= threshold) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Robô %d está longe da bola (distância: %.1f)", robot_id, distance);
    return BT::NodeStatus::FAILURE;
  }

  if (angle_threshold_deg > 0.0) {
    // Checagem de ângulo: a bola pode estar "perto" em distância e ainda assim fora do cone de
    // contato real do kicker (largura ~70-80mm do robô) se o robô não estiver de frente para
    // ela. Sem essa checagem, o chute às vezes acerta em cheio, às vezes de raspão (carrega a
    // bola de lado) e às vezes erra por completo — o mesmo "perto" em distância cobre todos
    // esses casos. Verificado no log: offset de 18.8° a 124mm resultou em chute inconsistente,
    // enquanto offset de 6.3° a 118mm chutou normalmente.
    double angle_to_ball_deg = std::atan2(dy, dx) * 180.0 / M_PI;
    double heading_deg = robot_orientation * 180.0 / M_PI;
    double offset_deg = normalizeAngleDeg(angle_to_ball_deg - heading_deg);

    if (std::abs(offset_deg) > angle_threshold_deg) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
        "Robô %d perto da bola mas desalinhado com a bola (dist: %.1f, offset: %.1fdeg > %.1fdeg)",
        robot_id, distance, offset_deg, angle_threshold_deg);
      return BT::NodeStatus::FAILURE;
    }

    // Se target_x e target_y forem fornecidos (A6), valida também se a direção do chute (heading)
    // está alinhada com o vetor bola -> alvo, evitando chutar na direção errada perto da mureta.
    double target_x = 0.0;
    double target_y = 0.0;
    if (getInput<double>("target_x", target_x) && getInput<double>("target_y", target_y)) {
      const double tdx = target_x - ball_x;
      const double tdy = target_y - ball_y;
      if (std::hypot(tdx, tdy) > 1.0) {
        const double angle_to_target_deg = std::atan2(tdy, tdx) * 180.0 / M_PI;
        const double target_offset_deg = normalizeAngleDeg(angle_to_target_deg - heading_deg);
        if (std::abs(target_offset_deg) > angle_threshold_deg) {
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
            "Robô %d perto da bola mas desalinhado com o alvo (dist: %.1f, target_offset: %.1fdeg > %.1fdeg)",
            robot_id, distance, target_offset_deg, angle_threshold_deg);
          return BT::NodeStatus::FAILURE;
        }
      }
    }
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "Robô %d está perto da bola (distância: %.1f)", robot_id, distance);
  return BT::NodeStatus::SUCCESS;
}

} // namespace oxebots_strategy
