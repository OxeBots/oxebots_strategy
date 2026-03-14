#include "oxebots_strategy/go_to_point_node.h"
#include <cmath>

namespace {
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
}

namespace oxebots_strategy {

GoToPointNode::GoToPointNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node) {
    // QoS Transient Local para garantir que o Planner receba o último Goal enviado
    auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);
    
    game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
        "/game_data", 10, std::bind(&GoToPointNode::gameDataCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "GoToPointNode pronto. Convertendo MM para Metros.");
}

BT::PortsList GoToPointNode::providedPorts() {
    return { BT::InputPort<unsigned int>("robot_id"),
             BT::InputPort<double>("x"),
             BT::InputPort<double>("y") };
}

void GoToPointNode::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg) {
    last_game_data_ = msg;
}

std::optional<oxebots_interfaces::msg::RobotGameData> GoToPointNode::getRobotData(unsigned int robot_id) {
    if (!last_game_data_) return std::nullopt;
    for (const auto& ally : last_game_data_->robots.allies) {
        if (ally.id == robot_id) return ally;
    }
    return std::nullopt;
}

void GoToPointNode::publishGoal() {
    double op_x, op_y;
    auto blackboard = config().blackboard;
    if (blackboard->get("opponent_goal_x", op_x) && blackboard->get("opponent_goal_y", op_y)) {
        target_w_ = std::atan2(op_y - target_pos_.y, op_x - target_pos_.x);
    } else {
        target_w_ = 0.0;
    }

    auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
    goal_msg->robot_id = robot_id_;
    goal_msg->pose.header.stamp = node_->now();
    goal_msg->pose.header.frame_id = "map"; 
    
    goal_msg->pose.pose.position.x = target_pos_.x / 1000.0;
    goal_msg->pose.pose.position.y = target_pos_.y / 1000.0;
    
    goal_msg->pose.pose.orientation.z = std::sin(target_w_ * 0.5);
    goal_msg->pose.pose.orientation.w = std::cos(target_w_ * 0.5);

    goal_pub_->publish(std::move(goal_msg));
}

BT::NodeStatus GoToPointNode::onStart() {
    if (!getInput<unsigned int>("robot_id", robot_id_)) return BT::NodeStatus::FAILURE;

    double tx, ty;
    if (!getInput<double>("x", tx) || !getInput<double>("y", ty)) return BT::NodeStatus::FAILURE;

    target_pos_.x = tx; 
    target_pos_.y = ty;

    publishGoal();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPointNode::onRunning() {
    // Verificar se o alvo mudou no Blackboard
    double tx, ty;
    if (getInput<double>("x", tx) && getInput<double>("y", ty)) {
        if (std::abs(tx - target_pos_.x) > 2.0 || std::abs(ty - target_pos_.y) > 2.0) {
            target_pos_.x = tx;
            target_pos_.y = ty;
            publishGoal();
        }
    }

    auto robot = getRobotData(robot_id_);
    if (!robot) return BT::NodeStatus::RUNNING;

    double dist = std::hypot(robot->x - target_pos_.x, robot->y - target_pos_.y);
    
    // Tolerâncias mais rígidas para garantir alinhamento (30mm e ~3 graus)
    bool pos_ok = (dist < 30.0);
    bool ori_ok = (std::abs(normalizeAngle(target_w_ - robot->orientation)) < 0.05);

    if (pos_ok && ori_ok) {
        RCLCPP_INFO(node_->get_logger(), "Robô %d chegou ao alvo com precisão.", robot_id_);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GoToPointNode::onHalted() {}

} // namespace oxebots_strategy