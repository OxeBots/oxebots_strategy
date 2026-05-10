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
  // Obter robot_id do InputPort
  if (!getInput<uint32_t>("robot_id", robot_id_)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing required input [robot_id] for GoalkeeperNode");
      return; // Ou lançar exceção, dependendo da política de erro
  }

  // Obter my_goal_x e is_yellow do blackboard
  auto blackboard = config.blackboard;
  if (!blackboard->get("my_goal_x", my_goal_x_)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing 'my_goal_x' from blackboard for GoalkeeperNode");
      return;
  }
  if (!blackboard->get("is_yellow", is_yellow_team_)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing 'is_yellow' from blackboard for GoalkeeperNode");
      return;
  }

  // Publisher para comandos do robô
  auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);

  // Subscriber para a posição da bola
  ball_subscription_ = node_->create_subscription<oxebots_interfaces::msg::BallPosition>(
    "/ball_data", 10, std::bind(&GoalkeeperNode::ball_callback, this, std::placeholders::_1));
  
  // Subscriber para os dados do jogo (para a posição do próprio robô)
  game_data_subscription_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10, std::bind(&GoalkeeperNode::game_data_callback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "GoalkeeperNode para o robô %d configurado. Meu gol em X: %.1f", robot_id_, my_goal_x_);
}

BT::PortsList GoalkeeperNode::providedPorts()
{
  return { BT::InputPort<uint32_t>("robot_id") };
}

void GoalkeeperNode::ball_callback(const oxebots_interfaces::msg::BallPosition::SharedPtr msg)
{
  last_ball_pos_ = *msg;
  ball_pos_updated_ = true;
}

void GoalkeeperNode::game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  // Encontrar os dados do próprio robô (goleiro)
  for (const auto& ally : msg->robots.allies) {
    if (ally.id == robot_id_) {
      current_robot_data_ = ally;
      robot_data_updated_ = true;
      return;
    }
  }
  robot_data_updated_ = false; // Se não encontrou os dados do robô
}

BT::NodeStatus GoalkeeperNode::onStart()
{
  // O goleiro é sempre o robô 0
  if (robot_id_ != 0) {
    RCLCPP_ERROR(node_->get_logger(), "GoalkeeperNode deve ser atribuído apenas ao robô 0. ID atual: %u", robot_id_);
    return BT::NodeStatus::FAILURE;
  }

  // Comando inicial: ir para a posição do gol (X do gol, Y=0)
  auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
  goal_msg->robot_id = robot_id_;
  
  goal_msg->pose.header.stamp = node_->now();
  goal_msg->pose.header.frame_id = "odom"; 
  goal_msg->pose.pose.position.x = my_goal_x_;
  goal_msg->pose.pose.position.y = 0.0; // Centro do gol
  
  // Orientação: olhar para o centro do campo (oposto ao gol)
  double target_w = normalizeAngle(std::atan2(0.0 - 0.0, (is_yellow_team_ ? -1.0 : 1.0) * 0.0 - my_goal_x_)); // Olhar para o centro do campo
  goal_msg->pose.pose.orientation.x = 0.0;
  goal_msg->pose.pose.orientation.y = 0.0;
  goal_msg->pose.pose.orientation.z = std::sin(target_w * 0.5);
  goal_msg->pose.pose.orientation.w = std::cos(target_w * 0.5);

  goal_pub_->publish(std::move(goal_msg));

  RCLCPP_INFO(node_->get_logger(), "GoalkeeperNode: Robô %d indo para a posição inicial do gol (%.1f, 0.0)", robot_id_, my_goal_x_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoalkeeperNode::onRunning()
{
  if (!ball_pos_updated_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "GoalkeeperNode: Posição da bola não atualizada. Aguardando...");
    return BT::NodeStatus::RUNNING; // Continua esperando a bola
  }

  // Obter a posição Y da bola
  double ball_y = last_ball_pos_.y;

  // Limitar a posição Y do alvo do goleiro entre -400mm e 400mm
  double target_y = std::clamp(ball_y, -400.0, 400.0);

  // Publicar comando de movimento
  auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
  goal_msg->robot_id = robot_id_;
  
  goal_msg->pose.header.stamp = node_->now();
  goal_msg->pose.header.frame_id = "odom"; 
  goal_msg->pose.pose.position.x = my_goal_x_; // X fixo na linha do gol
  goal_msg->pose.pose.position.y = target_y;
  
  // Orientação: olhar para o centro do campo (oposto ao gol)
  double target_w = normalizeAngle(std::atan2(0.0 - target_y, (is_yellow_team_ ? -1.0 : 1.0) * 0.0 - my_goal_x_)); // Olhar para o centro do campo
  goal_msg->pose.pose.orientation.x = 0.0;
  goal_msg->pose.pose.orientation.y = 0.0;
  goal_msg->pose.pose.orientation.z = std::sin(target_w * 0.5);
  goal_msg->pose.pose.orientation.w = std::cos(target_w * 0.5);

  goal_pub_->publish(std::move(goal_msg));

  return BT::NodeStatus::RUNNING;
}

void GoalkeeperNode::onHalted()
{
  // Opcional: Parar o robô ou enviar um comando de "manter posição"
  RCLCPP_INFO(node_->get_logger(), "GoalkeeperNode: Robô %d parado.", robot_id_);
}

}  // namespace oxebots_strategy
