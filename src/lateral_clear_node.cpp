#include "oxebots_strategy/lateral_clear_node.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>

namespace oxebots_strategy
{

static constexpr double LATERAL_TARGET_Y = 2000.0;
static constexpr double POS_UPDATE_THRESHOLD = 5.0;
static constexpr double ANG_UPDATE_THRESHOLD = 0.03;

// Decisão do chute (passe x afastar)
static constexpr double kNearGoalLineMm = 180.0;       // bola quase em cima da linha do gol
static constexpr double kPassMinDistMm = 300.0;
static constexpr double kPassMaxDistMm = 2500.0;
// Corredor livre de adversários: raio do robô (90) + raio da bola (21) + folga
static constexpr double kPassCorridorMm = 170.0;
static constexpr double kOpponentCloseMm = 300.0;      // adversário colado na bola: só afasta para a frente
static constexpr double kOpponentCloseHystMm = 80.0;
static constexpr double kClearKickSpeed = 3.0;         // m/s
static constexpr double kPassMinKickSpeed = 1.5;       // m/s
static constexpr double kPassArrivalSpeed = 1.0;       // m/s: velocidade com que a bola deve chegar ao aliado
static constexpr double kBallFrictionAccel = 0.8;      // m/s^2, mesmo valor do filtro de Kalman
// Erro angular máximo (rad) entre a orientação do robô (para onde a bola vai sair) e a direção do chute
static constexpr double kMaxClearAlignErr = 0.45;
static constexpr double kMaxPassAlignErr = 0.20;
// Em contato com a bola há mais que isso, a tolerância dobra para o goleiro não ficar ajustando por segundos
static constexpr double kAlignTimeoutS = 0.6;
static constexpr double kTimeoutErrFactor = 2.0;
// Se o robô se afasta da bola além disso, a direção do chute volta a ser recalculada
static constexpr double kPlanUnlockDistMm = 400.0;
static constexpr double kFieldMarginMm = 150.0;

LateralClearNode::LateralClearNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_(node_ptr)
{
    auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);

    game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
        "/game_data", 10,
        std::bind(&LateralClearNode::gameDataCallback, this, std::placeholders::_1));
}

void LateralClearNode::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_game_data_ = msg;
}

BT::PortsList LateralClearNode::providedPorts()
{
    return {
        BT::InputPort<uint32_t>("robot_id"),
        BT::InputPort<double>("ball_x"),
        BT::InputPort<double>("ball_y"),
        BT::InputPort<bool>("allow_leave_area", false,
                            "Permite o alvo fora da grande área (bola fora da área e mais perto do goleiro)"),
        BT::OutputPort<double>("kick_speed", "Velocidade do chute escolhida (m/s): passe ou afastar")
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
    last_pass_id_ = -1;
    opponent_close_ = false;
    plan_locked_ = false;
    contact_since_valid_ = false;

    return BT::NodeStatus::RUNNING;
}

// Escolhe para onde chutar. Padrão: afastar para a frente (com leve tendência para fora). Se não há
// adversário colado na bola e existe um aliado à frente com corredor livre de adversários, passa para ele.
LateralClearNode::KickPlan LateralClearNode::planKick(double ball_x, double ball_y, double my_goal_x)
{
    const double dir = (my_goal_x < 0) ? 1.0 : -1.0;

    // Para a frente; a componente lateral leva a bola para o lado em que ela já está
    const double target_side_y = (ball_y > 0) ? LATERAL_TARGET_Y : -LATERAL_TARGET_Y;
    double fx = dir;
    double fy = 0.25 * (target_side_y - ball_y) / LATERAL_TARGET_Y;
    const double fn = std::hypot(fx, fy);
    fx /= fn;
    fy /= fn;
    const KickPlan forward{fx, fy, kClearKickSpeed, -1};

    oxebots_interfaces::msg::GameData::SharedPtr game_data;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        game_data = last_game_data_;
    }
    if (!game_data) {
        last_pass_id_ = -1;
        return forward;
    }

    // Adversário muito perto da bola: não arrisca o passe
    double enemy_near = std::numeric_limits<double>::infinity();
    for (const auto & enemy : game_data->robots.enemies) {
        enemy_near = std::min(enemy_near, std::hypot(enemy.x - ball_x, enemy.y - ball_y));
    }
    opponent_close_ = enemy_near < (opponent_close_ ? kOpponentCloseMm + kOpponentCloseHystMm : kOpponentCloseMm);
    if (opponent_close_) {
        last_pass_id_ = -1;
        return forward;
    }

    const bool near_goal_line = std::abs(ball_x - my_goal_x) < kNearGoalLineMm;

    // Direção unitária e distância do passe ao aliado, se ele estiver à frente e sem adversário no corredor
    struct Pass { double ux; double uy; double dist; };
    auto evalPass = [&](const oxebots_interfaces::msg::RobotGameData & ally) -> std::optional<Pass> {
        const double dx = ally.x - ball_x;
        const double dy = ally.y - ball_y;
        const double dist = std::hypot(dx, dy);
        if (dist < kPassMinDistMm || dist > kPassMaxDistMm) return std::nullopt;
        const double ux = dx / dist;
        const double uy = dy / dist;
        if (dir * ux <= 0.0) return std::nullopt;                  // aliado atrás da bola
        if (near_goal_line && dir * ux < 0.8) return std::nullopt;   // na linha do gol só sai quase reto (travas)
        for (const auto & enemy : game_data->robots.enemies) {
            const double rx = enemy.x - ball_x;
            const double ry = enemy.y - ball_y;
            const double along = rx * ux + ry * uy;
            if (along <= 0.0 || along >= dist) continue;
            if (std::abs(rx * uy - ry * ux) < kPassCorridorMm) return std::nullopt;   // adversário entre os dois
        }
        return Pass{ux, uy, dist};
    };

    std::optional<Pass> best;
    int best_id = -1;

    // Mantém o aliado escolhido enquanto o passe continuar válido (evita trocar de alvo a cada tick)
    if (last_pass_id_ >= 0) {
        for (const auto & ally : game_data->robots.allies) {
            if (static_cast<int>(ally.id) == last_pass_id_ && ally.id != robot_id_) {
                best = evalPass(ally);
                if (best) best_id = last_pass_id_;
            }
        }
    }
    if (!best) {
        for (const auto & ally : game_data->robots.allies) {
            if (ally.id == robot_id_) continue;
            auto candidate = evalPass(ally);
            if (candidate && (!best || candidate->dist < best->dist)) {
                best = candidate;
                best_id = static_cast<int>(ally.id);
            }
        }
    }

    if (!best) {
        last_pass_id_ = -1;
        return forward;
    }

    last_pass_id_ = best_id;
    // v^2 = v_chegada^2 + 2*a*d: chega ao aliado ainda com velocidade, descontado o atrito
    const double dist_m = best->dist / 1000.0;
    const double speed = std::clamp(
        std::sqrt(kPassArrivalSpeed * kPassArrivalSpeed + 2.0 * kBallFrictionAccel * dist_m),
        kPassMinKickSpeed, kClearKickSpeed);
    return KickPlan{best->ux, best->uy, speed, best_id};
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

    bool allow_leave_area = false;
    (void)getInput<bool>("allow_leave_area", allow_leave_area);

    // Direção do chute: passe para um aliado livre ou afastar para a frente. Depois que o robô se posiciona
    // atrás da bola a direção fica travada, senão um aliado ou adversário se movendo faz o robô reajustar o
    // ângulo o tempo todo.
    if (plan_locked_ && std::hypot(ball_x - rx, ball_y - ry) > kPlanUnlockDistMm) plan_locked_ = false;
    const KickPlan plan = plan_locked_ ? locked_plan_ : planKick(ball_x, ball_y, my_goal_x);
    const double clear_dir_x = plan.dir_x;
    const double clear_dir_y = plan.dir_y;
    const double clear_angle = std::atan2(clear_dir_y, clear_dir_x);

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
    if (is_positioned_behind && !plan_locked_) {
        plan_locked_ = true;
        locked_plan_ = plan;
    }
    double offset_mm = is_positioned_behind ? kContactOffsetMm : kPreClearOffsetMm;

    double target_x = ball_x - clear_dir_x * offset_mm;
    double target_y = ball_y - clear_dir_y * offset_mm;

    // Se o robô estiver na frente da bola (lado do campo), abre lateralmente para contornar
    if (behind_proj < 20.0 && lateral_err < 150.0) {
        double detour_side = (ry >= ball_y) ? 1.0 : -1.0;
        target_y = ball_y + detour_side * 180.0;
    }

    // Limita o alvo para nunca sair da grande área, permitindo recuar até 180mm para dentro do gol.
    // Com allow_leave_area (bola fora da área e mais perto do goleiro) o limite do lado do campo passa a
    // ser o do próprio campo.
    constexpr double kMaxGoalDepthMm = 180.0;
    constexpr double kAreaMarginMm = 120.0;
    const double half_len = std::abs(my_goal_x);
    double field_width = 0.0;
    (void)config().blackboard->get("field_width", field_width);
    const double half_wid = (field_width > 0.0) ? field_width / 2.0 : p_width / 2.0;

    double min_x, max_x;
    if (my_goal_x < 0) {
        min_x = my_goal_x - kMaxGoalDepthMm;
        max_x = allow_leave_area ? half_len - kFieldMarginMm : my_goal_x + p_depth - kAreaMarginMm;
    } else {
        min_x = allow_leave_area ? -half_len + kFieldMarginMm : my_goal_x - p_depth + kAreaMarginMm;
        max_x = my_goal_x + kMaxGoalDepthMm;
    }
    double max_y = allow_leave_area ? half_wid - kFieldMarginMm : (p_width / 2.0) - kAreaMarginMm;

    target_x = std::clamp(target_x, min_x, max_x);
    target_y = std::clamp(target_y, -max_y, max_y);

    // Orienta o kicker frontal diretamente para a bola quando posicionado ou próximo
    double target_w = clear_angle;
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

    // Orientação do robô (a bola sai por onde o kicker aponta) contra a direção desejada do chute.
    // No passe a pontaria precisa ser mais fina; depois de kAlignTimeoutS em contato a tolerância dobra.
    const double kick_dir_err = std::abs(std::remainder(clear_angle - ryaw, 2.0 * M_PI));

    const bool contact_ready = behind_proj > 40.0 && dist_to_ball <= 150.0 && std::abs(kicker_ball_err) < 0.28;
    const rclcpp::Time now = node_->now();
    if (!contact_ready) {
        contact_since_valid_ = false;
    } else if (!contact_since_valid_) {
        contact_since_valid_ = true;
        contact_since_ = now;
    }
    const double contact_time = contact_since_valid_ ? (now - contact_since_).seconds() : 0.0;
    double max_kick_dir_err = (plan.pass_id >= 0) ? kMaxPassAlignErr : kMaxClearAlignErr;
    if (contact_time > kAlignTimeoutS) max_kick_dir_err *= kTimeoutErrFactor;

    // Autoriza o chute quando o goleiro está atrás da bola, em distância de contato e alinhado
    if (contact_ready && kick_dir_err < max_kick_dir_err) {
        if (plan.pass_id >= 0) {
            RCLCPP_INFO(node_->get_logger(),
                "Goleiro alinhado na bola para PASSAR ao robô %d: dist_ball=%.1fmm, erro_kicker=%.1fdeg, vel=%.2fm/s",
                plan.pass_id, dist_to_ball, kicker_ball_err * 180.0 / M_PI, plan.speed);
        } else {
            RCLCPP_INFO(node_->get_logger(),
                "Goleiro alinhado na bola para afastar: dist_ball=%.1fmm, erro_kicker=%.1fdeg",
                dist_to_ball, kicker_ball_err * 180.0 / M_PI);
        }
        setOutput("kick_speed", plan.speed);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void LateralClearNode::onHalted() {}

}
