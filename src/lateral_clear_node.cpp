#include "oxebots_strategy/lateral_clear_node.h"
#include "oxebots_strategy/goalkeeper_geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace oxebots_strategy
{

namespace gk = goalkeeper;

static constexpr double LATERAL_TARGET_Y = 2000.0;
static constexpr double POS_UPDATE_THRESHOLD = 5.0;
static constexpr double ANG_UPDATE_THRESHOLD = 0.03;

// Decisão do chute (passe x afastar)
static constexpr double kNearGoalLineMm = 180.0;       // bola quase em cima da linha do gol
static constexpr double kNearLineFanSpanRad = 35.0 * M_PI / 180.0;   // na linha do gol o leque é mais estreito (traves)
static constexpr double kPassMinDistMm = 300.0;
static constexpr double kPassMaxDistMm = 2500.0;
static constexpr double kOpponentCloseMm = 300.0;      // adversário colado na bola: só afasta para a frente
static constexpr double kOpponentCloseHystMm = 80.0;
// Posse nossa: um aliado (sem contar o goleiro) mais perto da bola que qualquer adversário, e nenhum adversário
// por perto. Com a posse, o goleiro não manda a bola para fora: aceita passe curto e lateral, e afasta com a
// velocidade que faz a bola parar dentro do campo.
static constexpr double kPossessionMarginMm = 100.0;
static constexpr double kPossessionEnemyFreeMm = 700.0;
static constexpr double kPassMinDistPossessionMm = 200.0;
static constexpr double kPossessionBackLimit = -0.2;   // cosseno mínimo do ângulo (na direção do ataque) do passe
static constexpr double kPossessionExitPenalty = 1.5;
static constexpr double kStopInsideMarginMm = 400.0;   // a bola deve parar pelo menos isto antes da borda
static constexpr double kPossessionMinClearSpeed = 1.0;   // m/s
static constexpr double kPassStickyScore = 0.15;       // o aliado anterior só é trocado se o novo for melhor por mais que isso
static constexpr double kClearKickSpeed = 3.0;         // m/s
static constexpr double kPassMinKickSpeed = 1.0;       // m/s
static constexpr double kPassArrivalSpeed = 1.0;       // m/s: velocidade com que a bola deve chegar ao aliado
static constexpr double kBallFrictionAccel = 0.8;      // m/s^2, mesmo valor do filtro de Kalman
// Erro angular máximo (rad) entre a orientação do robô (para onde a bola vai sair) e a direção do chute.
// Com o robô apontando para a direção do chute a tolerância pode ser pequena: a bola sai por onde ele aponta.
// O passe e a bola sobre a linha do gol (traves) pedem mais pontaria que afastar.
static constexpr double kMaxClearAlignErr = 0.35;
static constexpr double kMaxNearLineAlignErr = 0.25;
static constexpr double kMaxPassAlignErr = 0.20;
// Em contato com a bola há mais que isso, a tolerância dobra para o goleiro não ficar ajustando por segundos
static constexpr double kAlignTimeoutS = 0.4;
static constexpr double kTimeoutErrFactor = 2.0;
// Se o robô se afasta da bola além disso, a direção do chute volta a ser recalculada
static constexpr double kPlanUnlockDistMm = 400.0;
// A bola não pode ficar mais de 10 s na área de defesa: depois disto o goleiro desiste do passe e do capricho
static constexpr double kUrgentAfterS = 6.0;
static constexpr double kFieldMarginMm = 150.0;
// Folga além da coleira para o alvo do goleiro (a bola já está dentro da coleira)
static constexpr double kLeashTargetMarginMm = 100.0;

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
                            "Permite o alvo fora da grande área, até a coleira (bola fora da área e mais perto do goleiro)"),
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
    engage_start_ = node_->now();

    return BT::NodeStatus::RUNNING;
}

// Escolhe para onde chutar. Passe: o aliado à frente da bola com corredor livre de adversários (cápsula que
// cresce com a distância e pontas maiores em volta da bola e do receptor), de melhor escore. Sem passe seguro,
// ou com adversário colado na bola, ou com pressa (a bola não pode ficar mais de 10 s na área), afasta na
// melhor direção de um leque em torno do "para a frente" (maior folga de adversários, de preferência sem
// sair do campo).
LateralClearNode::KickPlan LateralClearNode::planKick(
    double ball_x, double ball_y, const goalkeeper::FieldGeometry & geo, bool urgent)
{
    const double dir = geo.forwardSign();
    const gk::Vec2 ball{ball_x, ball_y};
    const bool near_goal_line = std::abs(ball_x - geo.my_goal_x) < kNearGoalLineMm;

    // "Para a frente" com leve tendência para o lado onde a bola já está
    const double target_side_y = (ball_y > 0) ? LATERAL_TARGET_Y : -LATERAL_TARGET_Y;
    const double forward_angle = std::atan2(0.25 * (target_side_y - ball_y) / LATERAL_TARGET_Y, dir);

    oxebots_interfaces::msg::GameData::SharedPtr game_data;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        game_data = last_game_data_;
    }

    std::vector<gk::Vec2> opponents;
    if (game_data) {
        for (const auto & enemy : game_data->robots.enemies) {
            opponents.push_back({enemy.x, enemy.y});
        }
    }

    // Posse: algum aliado (fora o goleiro) mais perto da bola que qualquer adversário, e ninguém de lá por perto
    double ally_near_ball = std::numeric_limits<double>::infinity();
    double enemy_near_ball = std::numeric_limits<double>::infinity();
    if (game_data) {
        for (const auto & ally : game_data->robots.allies) {
            if (ally.id == robot_id_) continue;
            ally_near_ball = std::min(ally_near_ball, gk::distance({ally.x, ally.y}, ball));
        }
        for (const auto & opp : opponents) {
            enemy_near_ball = std::min(enemy_near_ball, gk::distance(opp, ball));
        }
    }
    const bool safe_possession = ally_near_ball + kPossessionMarginMm < enemy_near_ball &&
        enemy_near_ball > kPossessionEnemyFreeMm;

    gk::FanParams fan;
    if (near_goal_line) fan.half_span_rad = kNearLineFanSpanRad;
    if (safe_possession) fan.exit_penalty = kPossessionExitPenalty;
    auto clear_plan = [&]() {
        const gk::FanChoice choice = gk::chooseClearAngle(
            ball, forward_angle, opponents, geo.halfLength(), geo.halfWidth(), gk::PassParams(), fan);
        double speed = kClearKickSpeed;
        if (safe_possession) {
            // Com a posse, a bola tem de parar dentro do campo: velocidade pelo espaço que sobra até a borda
            const double room = gk::rayDistanceToField(ball, choice.angle, geo.halfLength(), geo.halfWidth()) -
                kStopInsideMarginMm;
            speed = gk::kickSpeedToStopWithin(room, kBallFrictionAccel, kPossessionMinClearSpeed, kClearKickSpeed);
        }
        return KickPlan{std::cos(choice.angle), std::sin(choice.angle), speed, -1};
    };

    if (!game_data || urgent) {
        last_pass_id_ = -1;
        return clear_plan();
    }

    // Adversário muito perto da bola: não arrisca o passe
    double enemy_near = std::numeric_limits<double>::infinity();
    for (const auto & opp : opponents) {
        enemy_near = std::min(enemy_near, gk::distance(opp, ball));
    }
    opponent_close_ = enemy_near < (opponent_close_ ? kOpponentCloseMm + kOpponentCloseHystMm : kOpponentCloseMm);
    if (opponent_close_) {
        last_pass_id_ = -1;
        return clear_plan();
    }

    // Aliados à frente da bola com corredor livre
    struct Candidate { int id; gk::Vec2 pos; double dist; double score; };
    std::vector<Candidate> candidates;
    const gk::PassParams pass{};
    for (const auto & ally : game_data->robots.allies) {
        if (ally.id == robot_id_) continue;
        const gk::Vec2 pos{ally.x, ally.y};
        const double dist = gk::distance(ball, pos);
        const double min_dist = safe_possession ? kPassMinDistPossessionMm : kPassMinDistMm;
        if (dist < min_dist || dist > kPassMaxDistMm) continue;
        const double ux = (pos.x - ball.x) / dist;
        // Aliado atrás da bola; com a posse aceita também o passe lateral (e levemente para trás)
        if (dir * ux <= (safe_possession ? kPossessionBackLimit : 0.0)) continue;
        if (near_goal_line && dir * ux < 0.8) continue;        // na linha do gol só sai quase reto (travas)
        const double clearance = gk::passClearance(ball, pos, opponents, pass);
        if (clearance <= 0.0) continue;                        // adversário no corredor ou colado em uma das pontas
        candidates.push_back({static_cast<int>(ally.id), pos, dist,
            gk::passScore(ball, pos, clearance, dir, geo.halfWidth(), kPassMaxDistMm)});
    }

    if (candidates.empty()) {
        last_pass_id_ = -1;
        return clear_plan();
    }

    const Candidate * best = &candidates.front();
    for (const auto & c : candidates) {
        if (c.score > best->score) best = &c;
    }
    // Mantém o aliado escolhido enquanto o passe continuar bom (evita trocar de alvo a cada tick)
    for (const auto & c : candidates) {
        if (c.id == last_pass_id_ && c.score >= best->score - kPassStickyScore) {
            best = &c;
            break;
        }
    }

    last_pass_id_ = best->id;
    const double ux = (best->pos.x - ball.x) / best->dist;
    const double uy = (best->pos.y - ball.y) / best->dist;
    // v^2 = v_chegada^2 + 2*a*d: chega ao aliado ainda com velocidade, descontado o atrito
    const double dist_m = best->dist / 1000.0;
    const double speed = std::clamp(
        std::sqrt(kPassArrivalSpeed * kPassArrivalSpeed + 2.0 * kBallFrictionAccel * dist_m),
        kPassMinKickSpeed, kClearKickSpeed);
    return KickPlan{ux, uy, speed, best->id};
}

BT::NodeStatus LateralClearNode::onRunning()
{
    double ball_x, ball_y;
    if (!getInput<double>("ball_x", ball_x) || !getInput<double>("ball_y", ball_y)) {
        return BT::NodeStatus::FAILURE;
    }

    // Geometria e pose vêm do Blackboard raiz; sem elas o nó espera (sem assumir valores padrão)
    gk::FieldGeometry geo;
    double rx, ry, ryaw;
    if (!gk::readFieldGeometry(config(), geo) || !gk::readRobotPose(config(), rx, ry, ryaw)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "LateralClear: aguardando geometria do campo e pose do robô no Blackboard raiz");
        return BT::NodeStatus::RUNNING;
    }
    const double my_goal_x = geo.my_goal_x;
    const double p_depth = geo.penalty_area_depth;
    const double p_width = geo.penalty_area_width;
    const double dir = geo.forwardSign();

    bool allow_leave_area = false;
    (void)getInput<bool>("allow_leave_area", allow_leave_area);

    const rclcpp::Time now = node_->now();
    const bool urgent = (now - engage_start_).seconds() > kUrgentAfterS;

    // Direção do chute: passe para um aliado livre ou afastar. Depois que o robô se posiciona atrás da bola a
    // direção fica travada, senão um aliado ou adversário se movendo faz o robô reajustar o ângulo o tempo todo.
    if (plan_locked_ && std::hypot(ball_x - rx, ball_y - ry) > kPlanUnlockDistMm) plan_locked_ = false;
    const KickPlan plan = plan_locked_ ? locked_plan_ : planKick(ball_x, ball_y, geo, urgent);
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
    // Com allow_leave_area (bola fora da área e mais perto do goleiro) o limite do lado do campo passa a ser
    // a coleira (fração da metade aliada), e o lateral o do próprio campo.
    constexpr double kMaxGoalDepthMm = 180.0;
    constexpr double kAreaMarginMm = 120.0;
    const double leash_x = my_goal_x + dir * (gk::kDefaultLeashFraction * geo.halfLength() + kLeashTargetMarginMm);

    double min_x, max_x;
    if (my_goal_x < 0) {
        min_x = my_goal_x - kMaxGoalDepthMm;
        max_x = allow_leave_area ? leash_x : my_goal_x + p_depth - kAreaMarginMm;
    } else {
        min_x = allow_leave_area ? leash_x : my_goal_x - p_depth + kAreaMarginMm;
        max_x = my_goal_x + kMaxGoalDepthMm;
    }
    const double max_y = allow_leave_area ? geo.halfWidth() - kFieldMarginMm : (p_width / 2.0) - kAreaMarginMm;

    target_x = std::clamp(target_x, min_x, max_x);
    target_y = std::clamp(target_y, -max_y, max_y);

    // Orientação: a bola sai por onde o kicker aponta. Já posicionado atrás da bola, o robô aponta para a direção
    // escolhida do chute (e não para a bola, que desviaria o chute pelo erro lateral); chegando perto, aponta
    // para a bola para ela entrar no cone do kicker.
    double target_w = clear_angle;
    if (is_positioned_behind) {
        target_w = clear_angle;
    } else if (dist_to_ball < 350.0) {
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
    // No passe a pontaria precisa ser mais fina; depois de kAlignTimeoutS em contato, ou com pressa
    // (10 s na área), a tolerância dobra.
    const double kick_dir_err = std::abs(std::remainder(clear_angle - ryaw, 2.0 * M_PI));

    const bool contact_ready = behind_proj > 40.0 && dist_to_ball <= 150.0 && std::abs(kicker_ball_err) < 0.28;
    if (!contact_ready) {
        contact_since_valid_ = false;
    } else if (!contact_since_valid_) {
        contact_since_valid_ = true;
        contact_since_ = now;
    }
    const double contact_time = contact_since_valid_ ? (now - contact_since_).seconds() : 0.0;
    const bool near_goal_line = std::abs(ball_x - my_goal_x) < kNearGoalLineMm;
    double max_kick_dir_err = (plan.pass_id >= 0) ? kMaxPassAlignErr :
        (near_goal_line ? kMaxNearLineAlignErr : kMaxClearAlignErr);
    if (contact_time > kAlignTimeoutS || urgent) max_kick_dir_err *= kTimeoutErrFactor;

    // Autoriza o chute quando o goleiro está atrás da bola, em distância de contato e alinhado
    if (contact_ready && kick_dir_err < max_kick_dir_err) {
        if (plan.pass_id >= 0) {
            RCLCPP_INFO(node_->get_logger(),
                "Goleiro alinhado na bola para PASSAR ao robô %d: dist_ball=%.1fmm, erro_kicker=%.1fdeg, vel=%.2fm/s",
                plan.pass_id, dist_to_ball, kicker_ball_err * 180.0 / M_PI, plan.speed);
        } else {
            RCLCPP_INFO(node_->get_logger(),
                "Goleiro alinhado na bola para afastar%s: dist_ball=%.1fmm, erro_kicker=%.1fdeg",
                urgent ? " (com pressa)" : "", dist_to_ball, kicker_ball_err * 180.0 / M_PI);
        }
        setOutput("kick_speed", plan.speed);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void LateralClearNode::onHalted() {}

}
