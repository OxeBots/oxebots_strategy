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
}

BT::PortsList IsBallCloseCondition::providedPorts()
{
  return { BT::InputPort<unsigned int>("robot_id"),
           BT::InputPort<double>("distance_threshold", 180.0, "Distância para considerar 'perto'"),
           BT::InputPort<double>("angle_threshold_deg", -1.0,
             "Desvio angular máximo (graus) entre a frente do robô e a bola, para o kicker "
             "realmente alcançar a bola. Se <= 0, não checa ângulo (comportamento antigo).") };
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
  if (!last_game_data_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Nenhum dado de jogo recebido.");
    return BT::NodeStatus::FAILURE;
  }

  double robot_x, robot_y, robot_orientation;
  bool robot_found = false;
  for (const auto& ally : last_game_data_->robots.allies) {
    if (ally.id == robot_id) {
      robot_x = ally.x;
      robot_y = ally.y;
      robot_orientation = ally.orientation;
      robot_found = true;
      break;
    }
  }

  if (!robot_found) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Robô com ID %d não encontrado.", robot_id);
    return BT::NodeStatus::FAILURE;
  }

  // Usa a posição REAL da bola (game_data), não a predita/rastreada do blackboard: este é o
  // gate final imediatamente antes do chute, então precisa refletir o instante atual, não uma
  // estimativa que pode ter alguns ms/mm de defasagem.
  double ball_x = last_game_data_->ball.x;
  double ball_y = last_game_data_->ball.y;

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
        "Robô %d perto da bola mas desalinhado (dist: %.1f, offset: %.1fdeg > %.1fdeg)",
        robot_id, distance, offset_deg, angle_threshold_deg);
      return BT::NodeStatus::FAILURE;
    }
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "Robô %d está perto da bola (distância: %.1f)", robot_id, distance);
  return BT::NodeStatus::SUCCESS;
}

} // namespace oxebots_strategy
