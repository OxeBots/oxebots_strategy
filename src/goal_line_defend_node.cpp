#include "oxebots_strategy/goal_line_defend_node.h"
#include "oxebots_strategy/goalkeeper_blackboard.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <utility>

namespace oxebots_strategy
{

static constexpr double GOALKEEPER_LINE_DEPTH = 300.0;
static constexpr double POSITION_UPDATE_THRESHOLD = 5.0;
static constexpr double ROBOT_RADIUS = 90.0;
// Folga entre o contorno do robô e as linhas da área, para o goleiro ficar inteiro dentro dela
static constexpr double kAreaMarginMm = 20.0;
// Abaixo dessa distância alvo->bola o rumo à bola fica instável: mantém a orientação anterior
static constexpr double kMinFacingDistMm = 100.0;
static constexpr double ANGLE_UPDATE_THRESHOLD = 0.05;
static constexpr double kMinShotSpeed = 450.0;
static constexpr double kMaxImpactTime = 3.0;
// Margem além das traves na detecção de chute (absorve ruído angular do Kalman em chutes no canto)
static constexpr double kShotGoalMarginMm = 150.0;
// Histerese da interceptação: limiares mais folgados para continuar interceptando um chute já detectado
static constexpr double kHoldMinShotSpeed = 300.0;
static constexpr double kHoldGoalMarginMm = 220.0;
static constexpr double kHoldImpactTimeMarginS = 0.5;
static constexpr double kInterceptHoldS = 0.25;
// Antecipação da bola na triangulação (passes laterais, cruzamentos e conduções em movimento)
static constexpr double kTriangMinLeadSpeedMmS = 200.0;
static constexpr double kTriangLeadS = 0.25;
static constexpr double kTriangMaxLeadMm = 450.0;
// Predição mais velha que isso (visão perdida) é descartada
static constexpr double kPredMaxAgeS = 0.15;

static std::optional<std::pair<double, double>> entryPoint(
    double bx, double by, double px, double py,
    double xmin, double xmax, double ymin, double ymax)
{
    const double d[2]  = {px - bx, py - by};
    const double o[2]  = {bx, by};
    const double lo[2] = {xmin, ymin};
    const double hi[2] = {xmax, ymax};
    double t0 = 0.0, t1 = 1.0;
    for (int i = 0; i < 2; ++i) {
        if (std::abs(d[i]) < 1e-9) {
            if (o[i] < lo[i] || o[i] > hi[i]) return std::nullopt;
        } else {
            double ta = (lo[i] - o[i]) / d[i];
            double tb = (hi[i] - o[i]) / d[i];
            if (ta > tb) std::swap(ta, tb);
            t0 = std::max(t0, ta);
            t1 = std::min(t1, tb);
            if (t0 > t1) return std::nullopt;
        }
    }
    return std::make_pair(bx + t0 * d[0], by + t0 * d[1]);
}

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
    const double line_x = my_goal_x_ + dir * GOALKEEPER_LINE_DEPTH;

    // Retângulo dos alvos: o lado do gol é a própria linha do gol (pode entrar no gol, para chutar a bola
    // sobre a linha); nos demais lados o robô inteiro (raio + folga) fica dentro da área
    const double x_far  = my_goal_x_ + dir * (area_depth - ROBOT_RADIUS - kAreaMarginMm);
    const double xmin   = std::min(my_goal_x_, x_far);
    const double xmax   = std::max(my_goal_x_, x_far);
    const double ylim   = area_width / 2.0 - ROBOT_RADIUS - kAreaMarginMm;
    const double post_y = std::min(half, ylim);

    // Leitura antecipada da cinemática da bola: usa /ball_predicted (Kalman) quando fresco e recorre
    // a ball_vx/ball_vy no blackboard raiz caso /ball_predicted esteja ausente ou expirado.
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

    // 1. Posição base: triangulação bola->postes na borda da área usando uma posição levemente
    // antecipada da bola quando ela está em movimento (acompanha passes laterais e dribles sem atraso).
    double tri_ball_x = ball_x;
    double tri_ball_y = ball_y;
    if (has_shot_kinematics) {
        const double ball_speed = std::hypot(shot_vx, shot_vy);
        if (ball_speed >= kTriangMinLeadSpeedMmS) {
            const double lead_dist = std::min(ball_speed * kTriangLeadS, kTriangMaxLeadMm);
            const double lead_scale = lead_dist / ball_speed;
            tri_ball_x = std::clamp(ball_x + shot_vx * lead_scale, -geo.halfLength(), geo.halfLength());
            tri_ball_y = std::clamp(ball_y + shot_vy * lead_scale, -geo.halfWidth(), geo.halfWidth());
            // Não projeta uma bola à frente do gol para trás da linha de fundo
            if (dir * (ball_x - my_goal_x_) > 0.0 && dir * (tri_ball_x - my_goal_x_) <= 0.0) {
                tri_ball_x = my_goal_x_ + dir * 1.0;
            }
        }
    }

    double target_x = line_x;
    double target_y = std::clamp(tri_ball_y, -half, half);
    const char * mode = "atras da linha do gol";

    if (dir * (ball_x - my_goal_x_) > 0.0) {   // bola à frente do gol
        // Se o gol for mais largo que o retângulo de movimento, mira nas bordas dele (os postes ficariam fora do
        // retângulo, as retas deixariam de cruzá-lo e o goleiro cairia no caso "segue a bola")
        auto e1 = entryPoint(tri_ball_x, tri_ball_y, my_goal_x_, -post_y, xmin, xmax, -ylim, ylim);
        auto e2 = entryPoint(tri_ball_x, tri_ball_y, my_goal_x_,  post_y, xmin, xmax, -ylim, ylim);
        if (e1 && e2) {
            target_x = 0.5 * (e1->first  + e2->first);
            target_y = 0.5 * (e1->second + e2->second);
            mode = "triangulacao";
        } else {
            // Retas não cruzam a área: segue a bola limitada à área
            target_x = std::clamp(tri_ball_x, xmin, xmax);
            target_y = std::clamp(tri_ball_y, -ylim, ylim);
            mode = "segue a bola (retas nao cruzam a area)";
        }
    }

    // 2. Interceptação de trajetória futura (chutes em direção ao gol), com margem de ruído e histerese
    bool shot = false;
    if (has_shot_kinematics && dir * shot_vx < 0.0 && dir * (shot_x - my_goal_x_) > 0.0) {
        const double min_speed = intercepting_ ? kHoldMinShotSpeed : kMinShotSpeed;
        const double y_margin  = intercepting_ ? kHoldGoalMarginMm : kShotGoalMarginMm;
        const double max_time  = intercepting_ ? kMaxImpactTime + kHoldImpactTimeMarginS : kMaxImpactTime;

        if (std::abs(shot_vx) >= min_speed) {
            auto t_goal = timeToReachXLinear(shot_x, shot_vx, my_goal_x_);
            if (t_goal && *t_goal > 0.0 && *t_goal <= max_time) {
                const double y_goal = shot_y + shot_vy * *t_goal;
                if (std::abs(y_goal) <= half + y_margin) {
                    const auto t_line = timeToReachXLinear(shot_x, shot_vx, line_x);
                    const double y_intercept = t_line ? (shot_y + shot_vy * *t_line) : y_goal;
                    // Clampeia nos limites dos postes para não sair do gol perseguindo chute para fora
                    last_intercept_y_ = std::clamp(y_intercept, -post_y, post_y);
                    intercept_hold_until_ = now + rclcpp::Duration::from_seconds(kInterceptHoldS);
                    intercepting_ = true;
                    shot = true;
                }
            }
        }
    }
    if (!shot && intercepting_) {
        // Mantém o último alvo de interceptação por um instante para não oscilar com o ruído da velocidade
        if (now < intercept_hold_until_) {
            shot = true;
        } else {
            intercepting_ = false;
        }
    }
    if (shot) {
        target_x = line_x;
        target_y = last_intercept_y_;
        mode = "interceptacao";
    }

    // 3. Mantém o alvo dentro da área
    target_x = std::clamp(target_x, xmin, xmax);
    target_y = std::clamp(target_y, -ylim, ylim);

    if (mode != last_mode_) {
        last_mode_ = mode;
        RCLCPP_INFO(node_->get_logger(),
            "GoalLineDefend: %s | alvo=(%.0f, %.0f) bola=(%.0f, %.0f) vel=(%.0f, %.0f) | gol=%.0f area=%.0fx%.0f x:[%.0f, %.0f] |y|<=%.0f",
            mode, target_x, target_y, ball_x, ball_y, shot_vx, shot_vy, geo.goal_width, area_depth, area_width, xmin, xmax, ylim);
    }

    // Orientação: kicker voltado para a bola (visto do ponto alvo). Com a bola muito perto do alvo o rumo
    // é instável, então mantém a orientação anterior (ou olha para o campo se ainda não houver uma).
    double target_w = (my_goal_x_ > 0) ? M_PI : 0.0;
    if (last_target_w_ > -99990.0) target_w = last_target_w_;
    const double ball_dx = ball_x - target_x;
    const double ball_dy = ball_y - target_y;
    if (std::hypot(ball_dx, ball_dy) > kMinFacingDistMm) target_w = std::atan2(ball_dy, ball_dx);

    const bool pos_changed = std::hypot(target_x - last_target_x_, target_y - last_target_y_) > POSITION_UPDATE_THRESHOLD;
    const bool ang_changed = std::abs(std::remainder(target_w - last_target_w_, 2.0 * M_PI)) > ANGLE_UPDATE_THRESHOLD;

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
