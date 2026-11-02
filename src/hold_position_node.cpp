#include "oxebots_strategy/hold_position_node.hpp"
#include <cmath>

namespace oxebots_strategy
{

HoldPositionNode::HoldPositionNode(const std::string& name, const BT::NodeConfig& config,
                                   rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  // Mesmo QoS das demais folhas de movimento: guarda o último objetivo.
  auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);
  game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
      "/game_data", 10,
      std::bind(&HoldPositionNode::gameDataCallback, this, std::placeholders::_1));
}

BT::PortsList HoldPositionNode::providedPorts()
{
  // O padrão "{robot_id}" lê a chave da blackboard quando o XML não passa a porta.
  return { BT::InputPort<unsigned int>("robot_id", "{robot_id}", "Id do robô") };
}

void HoldPositionNode::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_data_ = msg;
}

bool HoldPositionNode::publishHere()
{
  oxebots_interfaces::msg::GameData::SharedPtr data;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data = last_data_;
  }
  if (!data) return false;

  for (const auto& ally : data->robots.allies) {
    if (ally.id != robot_id_) continue;

    auto goal = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
    goal->robot_id = robot_id_;
    goal->pose.header.stamp = node_->now();
    goal->pose.header.frame_id = "map";
    goal->planner_type = oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE;
    goal->disable_arrival_latch = true;                       // nunca "trava" por chegada
    goal->pose.pose.position.x = ally.x / 1000.0;             // mm -> m
    goal->pose.pose.position.y = ally.y / 1000.0;
    goal->pose.pose.orientation.z = std::sin(ally.orientation * 0.5);
    goal->pose.pose.orientation.w = std::cos(ally.orientation * 0.5);
    goal_pub_->publish(std::move(goal));
    return true;
  }
  return false;
}

BT::NodeStatus HoldPositionNode::onStart()
{
  if (!getInput<unsigned int>("robot_id", robot_id_)) {
    RCLCPP_ERROR(node_->get_logger(), "HoldPosition: porta robot_id ausente");
    return BT::NodeStatus::FAILURE;                           // falhar alto, nunca em silêncio
  }
  if (!publishHere()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "HoldPosition: sem pose do robô %u", robot_id_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoldPositionNode::onRunning()
{
  // Renova o objetivo para manter o watchdog de 500 ms do SafetyGate ativo enquanto em RUNNING
  publishHere();
  return BT::NodeStatus::RUNNING;
}

void HoldPositionNode::onHalted()
{
  // Nada a limpar: não publicamos nenhum override. Idempotente por construção.
}

}  // namespace oxebots_strategy
