#include "oxebots_strategy/lateral_clear_node.h"
#include <cmath>
#include <memory>

namespace oxebots_strategy
{

static constexpr double LATERAL_TARGET_Y = 2000.0;
static constexpr double POS_UPDATE_THRESHOLD = 10.0;
static constexpr double ANG_UPDATE_THRESHOLD = 0.05;

LateralClearNode::LateralClearNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_(node_ptr)
{
    auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);
}

BT::PortsList LateralClearNode::providedPorts()
{
    return {
        BT::InputPort<uint32_t>("robot_id"),
        BT::InputPort<double>("ball_x"),
        BT::InputPort<double>("ball_y")
    };
}

BT::NodeStatus LateralClearNode::onStart()
{
    if (!getInput<uint32_t>("robot_id", robot_id_)) {
        robot_id_ = 0;
    }
    
    last_target_x_ = -99999.0;
    last_target_y_ = -99999.0;
    last_target_w_ = -99999.0;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LateralClearNode::onRunning()
{
    double ball_x, ball_y;
    if (!getInput<double>("ball_x", ball_x) || !getInput<double>("ball_y", ball_y)) {
        return BT::NodeStatus::FAILURE;
    }

    double rx, ry, ryaw;
    if (!config().blackboard->get("robot_x", rx) || 
        !config().blackboard->get("robot_y", ry) || 
        !config().blackboard->get("robot_yaw", ryaw)) {
        return BT::NodeStatus::RUNNING; // Wait for data
    }

    // Direção para afastar a bola: aponta em direção à linha lateral e para o meio-campo
    double target_side_y = (ball_y > 0) ? LATERAL_TARGET_Y : -LATERAL_TARGET_Y;
    double clear_dir_x = 0.0 - ball_x; // aponta para o centro/meio do campo
    double clear_dir_y = target_side_y - ball_y;
    double clear_norm = std::hypot(clear_dir_x, clear_dir_y);
    if (clear_norm > 1e-3) {
        clear_dir_x /= clear_norm;
        clear_dir_y /= clear_norm;
    } else {
        clear_dir_x = (ball_x < 0) ? 1.0 : -1.0;
        clear_dir_y = 0.0;
    }
    double target_w = std::atan2(clear_dir_y, clear_dir_x);

    // Posiciona o robô ATRÁS da bola na direção oposta ao chute (~115mm de distância)
    constexpr double kApproachOffsetMm = 115.0;
    double target_x = ball_x - clear_dir_x * kApproachOffsetMm;
    double target_y = ball_y - clear_dir_y * kApproachOffsetMm;

    // Limita o alvo para NUNCA sair da grande área (com margem para o raio do robô)
    double p_depth = 500.0, p_width = 1350.0, my_goal_x = -2200.0;
    (void)config().blackboard->get("penalty_area_depth", p_depth);
    (void)config().blackboard->get("penalty_area_width", p_width);
    (void)config().blackboard->get("my_goal_x", my_goal_x);

    constexpr double kAreaMarginMm = 120.0;
    double min_x, max_x;
    if (my_goal_x < 0) {
        min_x = my_goal_x + kAreaMarginMm;
        max_x = my_goal_x + p_depth - kAreaMarginMm;
    } else {
        min_x = my_goal_x - p_depth + kAreaMarginMm;
        max_x = my_goal_x - kAreaMarginMm;
    }
    double max_y = (p_width / 2.0) - kAreaMarginMm;

    target_x = std::clamp(target_x, min_x, max_x);
    target_y = std::clamp(target_y, -max_y, max_y);

    // Calcula distância direta até a bola e ângulo da bola
    double dx_ball = ball_x - rx;
    double dy_ball = ball_y - ry;
    double dist_to_ball = std::hypot(dx_ball, dy_ball);
    double angle_to_ball = std::atan2(dy_ball, dx_ball);

    // Quando próximo da bola, orienta a frente do robô diretamente para a bola para garantir contato do kicker
    if (dist_to_ball < 350.0) {
        target_w = angle_to_ball;
    }

    bool target_changed = (std::abs(target_x - last_target_x_) > POS_UPDATE_THRESHOLD || 
                           std::abs(target_y - last_target_y_) > POS_UPDATE_THRESHOLD ||
                           std::abs(target_w - last_target_w_) > ANG_UPDATE_THRESHOLD);

    if (target_changed) {
        auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
        goal_msg->robot_id = robot_id_;
        // Linha reta direta até a bola para não ser repelido pelo D* como obstáculo
        goal_msg->planner_type = oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE;
        goal_msg->pose.header.stamp = node_->now();
        goal_msg->pose.header.frame_id = "map";
        goal_msg->pose.pose.position.x = target_x / 1000.0;
        goal_msg->pose.pose.position.y = target_y / 1000.0;
        goal_msg->pose.pose.orientation.z = std::sin(target_w * 0.5);
        goal_msg->pose.pose.orientation.w = std::cos(target_w * 0.5);
        
        goal_pub_->publish(std::move(goal_msg));

        last_target_x_ = target_x;
        last_target_y_ = target_y;
        last_target_w_ = target_w;
    }

    // Ângulo real entre o kicker frontal e a bola
    double kicker_ball_err = angle_to_ball - ryaw;
    while (kicker_ball_err > M_PI) kicker_ball_err -= 2.0 * M_PI;
    while (kicker_ball_err < -M_PI) kicker_ball_err += 2.0 * M_PI;

    // Só autoriza o chute quando o robô estiver a distância de contato (~140mm) e com o kicker alinhado (< 12 graus)
    if (dist_to_ball <= 140.0 && std::abs(kicker_ball_err) < 0.20) {
        RCLCPP_INFO(node_->get_logger(),
            "Goleiro alinhado na bola para afastar: dist_ball=%.1fmm, erro_kicker=%.1fdeg",
            dist_to_ball, kicker_ball_err * 180.0 / M_PI);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void LateralClearNode::onHalted() {}

}
