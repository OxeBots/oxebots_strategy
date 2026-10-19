#include "oxebots_strategy/lateral_clear_node.h"
#include <algorithm>
#include <cmath>
#include <memory>

namespace oxebots_strategy
{

static constexpr double LATERAL_TARGET_Y = 2000.0;
static constexpr double POS_UPDATE_THRESHOLD = 5.0;
static constexpr double ANG_UPDATE_THRESHOLD = 0.03;

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

    double p_depth = 500.0, p_width = 1350.0, my_goal_x = -2200.0;
    (void)config().blackboard->get("penalty_area_depth", p_depth);
    (void)config().blackboard->get("penalty_area_width", p_width);
    (void)config().blackboard->get("my_goal_x", my_goal_x);

    // Direção para afastar a bola:
    // Quando a bola está em cima/perto da linha do gol, aponta prioritariamente para frente (meio-campo)
    // para o goleiro entrar reto atrás da bola dentro do gol e chutar para fora sem bater na trave.
    double target_side_y = (ball_y > 0) ? LATERAL_TARGET_Y : -LATERAL_TARGET_Y;
    double clear_dir_x = 0.0 - ball_x;
    double clear_dir_y = target_side_y - ball_y;
    if (std::abs(ball_x - my_goal_x) < 180.0) {
        clear_dir_x = (my_goal_x < 0) ? 1.0 : -1.0;
        clear_dir_y = 0.25 * (target_side_y - ball_y) / LATERAL_TARGET_Y;
    }

    double clear_norm = std::hypot(clear_dir_x, clear_dir_y);
    if (clear_norm > 1e-3) {
        clear_dir_x /= clear_norm;
        clear_dir_y /= clear_norm;
    } else {
        clear_dir_x = (my_goal_x < 0) ? 1.0 : -1.0;
        clear_dir_y = 0.0;
    }

    // Projeção da posição do robô em relação à bola ao longo da linha de chute:
    // behind_proj > 0 significa que o robô já está ATRÁS da bola (do lado do gol).
    double vec_r_to_ball_x = ball_x - rx;
    double vec_r_to_ball_y = ball_y - ry;
    double behind_proj = vec_r_to_ball_x * clear_dir_x + vec_r_to_ball_y * clear_dir_y;
    double lateral_err = std::abs(-vec_r_to_ball_x * clear_dir_y + vec_r_to_ball_y * clear_dir_x);

    double dist_to_ball = std::hypot(vec_r_to_ball_x, vec_r_to_ball_y);
    double angle_to_ball = std::atan2(vec_r_to_ball_y, vec_r_to_ball_x);

    // Aproximação em duas fases exclusivamente para o goleiro:
    // - Fase 1 (posicionar atrás da bola): vai para 125mm atrás da bola (contornando se estiver na frente).
    // - Fase 2 (alinhar e encostar para o chute): já atrás da bola, avança o alvo para 55mm da bola.
    //   Isso mantém check_dist entre 40mm e 80mm no PathFollowerNode (acima de 10mm para NUNCA desligar
    //   o giro com arrived=true, e abaixo de 100mm para girar no lugar antes de encostar na bola).
    constexpr double kPreClearOffsetMm = 125.0;
    constexpr double kContactOffsetMm = 55.0;

    bool is_positioned_behind = (behind_proj >= 65.0 && lateral_err <= 65.0);
    double offset_mm = is_positioned_behind ? kContactOffsetMm : kPreClearOffsetMm;

    double target_x = ball_x - clear_dir_x * offset_mm;
    double target_y = ball_y - clear_dir_y * offset_mm;

    // Se o robô estiver na frente da bola (lado do campo), abre lateralmente para contornar
    if (behind_proj < 20.0 && lateral_err < 150.0) {
        double detour_side = (ry >= ball_y) ? 1.0 : -1.0;
        target_y = ball_y + detour_side * 180.0;
    }

    // Limita o alvo para nunca sair da grande área, permitindo recuar até 180mm para dentro do gol
    constexpr double kMaxGoalDepthMm = 180.0;
    constexpr double kAreaMarginMm = 120.0;
    double min_x, max_x;
    if (my_goal_x < 0) {
        min_x = my_goal_x - kMaxGoalDepthMm;
        max_x = my_goal_x + p_depth - kAreaMarginMm;
    } else {
        min_x = my_goal_x - p_depth + kAreaMarginMm;
        max_x = my_goal_x + kMaxGoalDepthMm;
    }
    double max_y = (p_width / 2.0) - kAreaMarginMm;

    target_x = std::clamp(target_x, min_x, max_x);
    target_y = std::clamp(target_y, -max_y, max_y);

    // Orienta o kicker frontal diretamente para a bola quando posicionado ou próximo
    double target_w = std::atan2(clear_dir_y, clear_dir_x);
    if (is_positioned_behind || dist_to_ball < 350.0) {
        target_w = angle_to_ball;
    }

    bool target_changed = (std::abs(target_x - last_target_x_) > POS_UPDATE_THRESHOLD ||
                           std::abs(target_y - last_target_y_) > POS_UPDATE_THRESHOLD ||
                           std::abs(target_w - last_target_w_) > ANG_UPDATE_THRESHOLD);

    if (target_changed) {
        auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
        goal_msg->robot_id = robot_id_;
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

    // Autoriza o chute quando o goleiro está atrás da bola, em distância de contato e alinhado
    if (behind_proj > 40.0 && dist_to_ball <= 150.0 && std::abs(kicker_ball_err) < 0.28) {
        RCLCPP_INFO(node_->get_logger(),
            "Goleiro alinhado na bola para afastar: dist_ball=%.1fmm, erro_kicker=%.1fdeg",
            dist_to_ball, kicker_ball_err * 180.0 / M_PI);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void LateralClearNode::onHalted() {}

}
