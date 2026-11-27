#include "oxebots_strategy/goal_line_defend_node.h"
#include "oxebots_strategy/goalkeeper_blackboard.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <utility>

namespace oxebots_strategy
{

static constexpr double BASE_DEFEND_LINE_DEPTH = 200.0;
static constexpr double MAX_DEFEND_LINE_DEPTH = 260.0;
static constexpr double POSITION_UPDATE_THRESHOLD = 15.0;
static constexpr double INTERCEPT_UPDATE_THRESHOLD = 5.0;
static constexpr double ROBOT_RADIUS = 90.0;
// Folga entre o contorno do robô e as linhas da área, para o goleiro ficar inteiro dentro dela
static constexpr double kAreaMarginMm = 20.0;
// Abaixo dessa distância alvo->bola o rumo à bola fica instável: mantém a orientação anterior
static constexpr double kMinFacingDistMm = 100.0;
static constexpr double ANGLE_UPDATE_THRESHOLD = 0.10;
static constexpr double INTERCEPT_ANGLE_THRESHOLD = 0.05;
static constexpr double kMinShotSpeed = 380.0;
static constexpr double kMaxImpactTime = 3.0;
// Margem além das traves na detecção de chute (absorve ruído angular do Kalman/visão em chutes no canto)
static constexpr double kShotGoalMarginMm = 160.0;
// Histerese da interceptação: limiares mais folgados para continuar interceptando um chute já detectado
static constexpr double kHoldMinShotSpeed = 260.0;
static constexpr double kHoldGoalMarginMm = 230.0;
static constexpr double kHoldImpactTimeMarginS = 0.5;
static constexpr double kInterceptHoldS = 0.25;
// Antecipação da bola no posicionamento base (passes laterais e conduções em movimento)
static constexpr double kTriangMinLeadSpeedMmS = 200.0;
static constexpr double kTriangLeadS = 0.25;
static constexpr double kTriangMaxLeadMm = 350.0;
// Suavização contra ruído de câmera em visão real
static constexpr double kBallPosEmaAlpha = 0.45;
static constexpr double kBallSnapJumpMm = 250.0;
// Predição mais velha que isso (visão perdida) é descartada
static constexpr double kPredMaxAgeS = 0.15;

// Menor t >= 0 em que p + v*t alcança "target" em trajetória retilínea.
// Como o atrito de rolamento no carpete atua sempre antiparalelo à velocidade, o ponto Y onde a bola
// cruza a linha do gol/defesa independe do coeficiente de atrito; usar o raio linear evita descartar
// chutes médios (0,9–1,8 m/s) quando o atrito assumido pelo Kalman (800 mm/s^2) é maior que o do campo.
static std::optional<double> timeToReachXLinear(double p, double v, double target)
{
    if (std::abs(v) < 1e-9) return std::nullopt;
    const double t = (target - p) / v;
    if (t < 0.0) return std::nullopt;
    return t;
}

GoalLineDefendNode::GoalLineDefendNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_(node_ptr)
{
    auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);

    ball_pred_sub_ = node_->create_subscription<oxebots_interfaces::msg::BallPrediction>(
        "/ball_predicted", 10,
        std::bind(&GoalLineDefendNode::ballPredictionCallback, this, std::placeholders::_1));
}

void GoalLineDefendNode::ballPredictionCallback(const oxebots_interfaces::msg::BallPrediction::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_ball_pred_ = msg;
    last_ball_pred_stamp_ = node_->now();
}

BT::PortsList GoalLineDefendNode::providedPorts()
{
    return {
        BT::InputPort<uint32_t>("robot_id"),
        BT::InputPort<double>("ball_x"),
        BT::InputPort<double>("ball_y")
    };
}

BT::NodeStatus GoalLineDefendNode::onStart()
{
    if (!getInput<uint32_t>("robot_id", robot_id_)) {
        robot_id_ = 0;
    }

    last_target_y_ = -99999.0;
    last_target_x_ = -99999.0;
    last_target_w_ = -99999.0;

    // Descarta predição de execuções anteriores do nó e qualquer interceptação em andamento
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_ball_pred_.reset();
    }
    intercepting_ = false;
    has_filtered_ball_ = false;
    return onRunning();
}

BT::NodeStatus GoalLineDefendNode::onRunning()
{
    double ball_x, ball_y;
    if (!getInput<double>("ball_x", ball_x) || !getInput<double>("ball_y", ball_y)) return BT::NodeStatus::FAILURE;

    // Geometria atualizada por /field_geometry no Blackboard raiz; sem ela o nó falha, em vez de assumir
    // valores padrão (que poderiam inverter o lado do gol ou usar uma área de outro campo)
    goalkeeper::FieldGeometry geo;
    if (!goalkeeper::readFieldGeometry(config(), geo)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "GoalLineDefend: aguardando a geometria do campo no Blackboard raiz");
        return BT::NodeStatus::FAILURE;
    }
    my_goal_x_ = geo.my_goal_x;
    const double area_depth = geo.penalty_area_depth;
    const double area_width = geo.penalty_area_width;
    const double half   = geo.goal_width / 2.0;
    const double dir    = (my_goal_x_ < 0) ? 1.0 : -1.0;   // +1 se o gol está à esquerda

    // Faixa segura dentro da área e próxima da linha do gol: nunca sai da área nem avança colado na borda frontal
    const double min_line_depth = ROBOT_RADIUS + kAreaMarginMm;
    const double max_area_depth = std::max(min_line_depth, area_depth - ROBOT_RADIUS - kAreaMarginMm);
    const double max_defend_depth = std::min(max_area_depth, MAX_DEFEND_LINE_DEPTH);
    const double base_depth = std::clamp(BASE_DEFEND_LINE_DEPTH, min_line_depth, max_defend_depth);
    const double line_x = my_goal_x_ + dir * base_depth;

    const double x_near = my_goal_x_ + dir * min_line_depth;
    const double x_far  = my_goal_x_ + dir * max_defend_depth;
    const double xmin   = std::min(x_near, x_far);
    const double xmax   = std::max(x_near, x_far);
    const double ylim   = std::max(0.0, area_width / 2.0 - ROBOT_RADIUS - kAreaMarginMm);
    const double post_y = std::min(half, ylim);

    // Cinemática da bola: prefere /ball_predicted quando fresco (já sem frames repetidos),
    // usando ball_vx/ball_vy da blackboard raiz como fallback imediato.
    oxebots_interfaces::msg::BallPrediction::SharedPtr pred;
    rclcpp::Time pred_stamp(0, 0, RCL_ROS_TIME);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pred = last_ball_pred_;
        pred_stamp = last_ball_pred_stamp_;
    }

    const rclcpp::Time now = node_->now();
    const bool pred_fresh = pred && (now - pred_stamp).seconds() <= kPredMaxAgeS;

    double shot_x = ball_x;
    double shot_y = ball_y;
    double shot_vx = 0.0;
    double shot_vy = 0.0;
    bool has_shot_kinematics = false;

    if (pred_fresh && std::isfinite(pred->x) && std::isfinite(pred->y) &&
        std::isfinite(pred->vx) && std::isfinite(pred->vy)) {
        shot_x = pred->x;
        shot_y = pred->y;
        shot_vx = pred->vx;
        shot_vy = pred->vy;
        has_shot_kinematics = true;
    } else if (goalkeeper::readBallVelocity(config(), shot_vx, shot_vy)) {
        has_shot_kinematics = true;
    }

    const double ball_speed = has_shot_kinematics ? std::hypot(shot_vx, shot_vy) : 0.0;

    // 1. Posição base: cobertura rápida na linha do gol com suavização contra ruído de visão
    if (!has_filtered_ball_ || std::hypot(ball_x - filt_ball_x_, ball_y - filt_ball_y_) > kBallSnapJumpMm ||
        ball_speed > 500.0) {
        filt_ball_x_ = ball_x;
        filt_ball_y_ = ball_y;
        has_filtered_ball_ = true;
    } else {
        filt_ball_x_ = kBallPosEmaAlpha * ball_x + (1.0 - kBallPosEmaAlpha) * filt_ball_x_;
        filt_ball_y_ = kBallPosEmaAlpha * ball_y + (1.0 - kBallPosEmaAlpha) * filt_ball_y_;
    }

    double tri_ball_x = filt_ball_x_;
    double tri_ball_y = filt_ball_y_;
    if (has_shot_kinematics && ball_speed >= kTriangMinLeadSpeedMmS) {
        const double lead_dist = std::min(ball_speed * kTriangLeadS, kTriangMaxLeadMm);
        const double lead_scale = lead_dist / ball_speed;
        tri_ball_x = std::clamp(ball_x + shot_vx * lead_scale, -geo.halfLength(), geo.halfLength());
        tri_ball_y = std::clamp(ball_y + shot_vy * lead_scale, -geo.halfWidth(), geo.halfWidth());
        if (dir * (ball_x - my_goal_x_) > 0.0 && dir * (tri_ball_x - my_goal_x_) <= 0.0) {
            tri_ball_x = my_goal_x_ + dir * 1.0;
        }
    }

    double target_x = line_x;
    double target_y = std::clamp(tri_ball_y, -post_y, post_y);
    const char * mode = "atras da linha do gol";

    if (dir * (ball_x - my_goal_x_) > 0.0) {
        // Projeção da linha gol->bola sobre a linha de defesa (suave e contínua, sem saltos nos cantos da área)
        const double dx_ball = std::max(std::abs(tri_ball_x - my_goal_x_), 50.0);
        const double frac = std::clamp(base_depth / dx_ball, 0.0, 1.0);
        target_x = line_x;
        target_y = std::clamp(tri_ball_y * frac, -post_y, post_y);
        mode = "linha de gol reativa";
    }

    // 2. Interceptação perpendicular na trajetória do chute (estilo ShadowBall::computeShotBlockTarget),
    //    mantida estritamente dentro da área e limitada a MAX_DEFEND_LINE_DEPTH.
    bool shot = false;
    if (has_shot_kinematics && dir * shot_vx < 0.0 && dir * (shot_x - my_goal_x_) > 0.0) {
        const double min_speed = intercepting_ ? kHoldMinShotSpeed : kMinShotSpeed;
        const double y_margin  = intercepting_ ? kHoldGoalMarginMm : kShotGoalMarginMm;
        const double max_time  = intercepting_ ? kMaxImpactTime + kHoldImpactTimeMarginS : kMaxImpactTime;

        if (std::abs(shot_vx) >= min_speed) {
            auto t_goal = timeToReachXLinear(shot_x, shot_vx, my_goal_x_);
            if (t_goal && *t_goal > 0.01 && *t_goal <= max_time) {
                const double y_goal = shot_y + shot_vy * *t_goal;
                if (std::abs(y_goal) <= half + y_margin) {
                    double rx = line_x;
                    double ry = std::clamp(y_goal, -post_y, post_y);
                    double ryaw = 0.0;
                    (void)goalkeeper::readRobotPose(config(), rx, ry, ryaw);

                    const double seg_dx = my_goal_x_ - shot_x;
                    const double seg_dy = y_goal - shot_y;
                    const double seg_len = std::hypot(seg_dx, seg_dy);
                    double proj_x = line_x;
                    if (seg_len > 10.0) {
                        const double ux = seg_dx / seg_len;
                        const double uy = seg_dy / seg_len;
                        const double along = std::clamp(
                            (rx - shot_x) * ux + (ry - shot_y) * uy, 0.0, seg_len);
                        proj_x = shot_x + ux * along;
                    }

                    const double ix = std::clamp(proj_x, xmin, xmax);
                    const auto t_hit = timeToReachXLinear(shot_x, shot_vx, ix);
                    const double y_hit = t_hit ? (shot_y + shot_vy * *t_hit) : y_goal;

                    last_intercept_x_ = ix;
                    last_intercept_y_ = std::clamp(y_hit, -post_y, post_y);
                    intercept_hold_until_ = now + rclcpp::Duration::from_seconds(kInterceptHoldS);
                    intercepting_ = true;
                    shot = true;
                }
            }
        }
    }
    if (!shot && intercepting_) {
        if (now < intercept_hold_until_) {
            shot = true;
        } else {
            intercepting_ = false;
        }
    }
    if (shot) {
        target_x = last_intercept_x_;
        target_y = last_intercept_y_;
        mode = "interceptacao perpendicular";
    }

    // 3. Mantém o alvo estritamente dentro da zona segura da área
    target_x = std::clamp(target_x, xmin, xmax);
    target_y = std::clamp(target_y, -post_y, post_y);

    if (mode != last_mode_) {
        last_mode_ = mode;
        RCLCPP_INFO(node_->get_logger(),
            "GoalLineDefend: %s | alvo=(%.0f, %.0f) bola=(%.0f, %.0f) vel=(%.0f, %.0f) | gol=%.0f area=%.0fx%.0f x:[%.0f, %.0f] |y|<=%.0f",
            mode, target_x, target_y, ball_x, ball_y, shot_vx, shot_vy, geo.goal_width, area_depth, area_width, xmin, xmax, post_y);
    }

    // Orientação: kicker voltado para a bola (visto do ponto alvo). Com a bola muito perto do alvo o rumo
    // é instável, então mantém a orientação anterior (ou olha para o campo se ainda não houver uma).
    double target_w = (my_goal_x_ > 0) ? M_PI : 0.0;
    if (last_target_w_ > -99990.0) target_w = last_target_w_;
    const double ball_dx = ball_x - target_x;
    const double ball_dy = ball_y - target_y;
    if (std::hypot(ball_dx, ball_dy) > kMinFacingDistMm) target_w = std::atan2(ball_dy, ball_dx);

    const double pos_thresh = shot ? INTERCEPT_UPDATE_THRESHOLD : POSITION_UPDATE_THRESHOLD;
    const double ang_thresh = shot ? INTERCEPT_ANGLE_THRESHOLD : ANGLE_UPDATE_THRESHOLD;
    const bool pos_changed = std::hypot(target_x - last_target_x_, target_y - last_target_y_) > pos_thresh;
    const bool ang_changed = std::abs(std::remainder(target_w - last_target_w_, 2.0 * M_PI)) > ang_thresh;

    // Reenvia o objetivo a cada 200 ms mesmo sem mudança: o watchdog de 500 ms do SafetyGate zera
    // o robô se nenhum /robot_goal ou override chegar (o GoToPoint e o GoalkeeperNode já fazem isso).
    constexpr double kGoalHeartbeatS = 0.2;
    const double now_s = now.seconds();
    const bool heartbeat_due = (now_s - last_goal_publish_s_) >= kGoalHeartbeatS;

    if (pos_changed || ang_changed || heartbeat_due) {
        auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
        goal_msg->robot_id = robot_id_;
        goal_msg->planner_type = oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE;
        goal_msg->disable_arrival_latch = true;
        goal_msg->pose.header.stamp = node_ -> now();
        goal_msg->pose.header.frame_id = "map";
        goal_msg->pose.pose.position.x = target_x / 1000.0;
        goal_msg->pose.pose.position.y = target_y / 1000.0;

        goal_msg->pose.pose.orientation.z = std::sin(target_w * 0.5);
        goal_msg->pose.pose.orientation.w = std::cos(target_w * 0.5);

        goal_pub_->publish(std::move(goal_msg));
        last_target_y_ = target_y;
        last_target_x_ = target_x;
        last_target_w_ = target_w;
        last_goal_publish_s_ = now_s;
    }

    return BT::NodeStatus::RUNNING;
}

void GoalLineDefendNode::onHalted() {}

}
