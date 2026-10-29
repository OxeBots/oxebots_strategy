#include "oxebots_strategy/goal_line_defend_node.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <utility>

namespace oxebots_strategy
{

static constexpr double DEFAULT_GOAL_WIDTH = 800.0;
static constexpr double GOALKEEPER_LINE_DEPTH = 300.0;
static constexpr double POSITION_UPDATE_THRESHOLD = 5.0;
static constexpr double ROBOT_RADIUS = 90.0;
// Folga entre o contorno do robô e as linhas da área, para o goleiro ficar inteiro dentro dela
static constexpr double kAreaMarginMm = 20.0;
// Abaixo dessa distância alvo->bola o rumo à bola fica instável: mantém a orientação anterior
static constexpr double kMinFacingDistMm = 100.0;
static constexpr double ANGLE_UPDATE_THRESHOLD = 0.05;
// Fallbacks da geometria (campo Entry Level); o valor real vem de /field_geometry via Blackboard
static constexpr double DEFAULT_AREA_DEPTH = 500.0;
static constexpr double DEFAULT_AREA_WIDTH = 1350.0;
static constexpr double kMinShotSpeed = 500.0;
static constexpr double kMaxImpactTime = 3.0;
// Histerese da interceptação: limiares mais folgados para continuar interceptando um chute já detectado
static constexpr double kHoldMinShotSpeed = 300.0;
static constexpr double kHoldGoalMarginMm = 60.0;
static constexpr double kHoldImpactTimeMarginS = 0.5;
static constexpr double kInterceptHoldS = 0.25;
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

// Menor t >= 0 em que p + v*t + a*t^2/2 alcança "target". Considera o atrito (a oposta a v): se a bola
// para antes de chegar, ou só chegaria revertendo o movimento, devolve nullopt.
static std::optional<double> timeToReachX(double p, double v, double a, double target)
{
    const double d = target - p;
    if (std::abs(a) < 1e-6) {
        if (std::abs(v) < 1e-9) return std::nullopt;
        const double t = d / v;
        if (t < 0.0) return std::nullopt;
        return t;
    }
    const double disc = v * v + 2.0 * a * d;
    if (disc < 0.0) return std::nullopt;
    const double sq = std::sqrt(disc);
    double t = std::numeric_limits<double>::infinity();
    for (const double r : {(-v + sq) / a, (-v - sq) / a}) {
        if (r >= 0.0 && r < t) t = r;
    }
    if (std::isinf(t)) return std::nullopt;
    if (a * v < 0.0 && t > -v / a) return std::nullopt;   // depois de a bola parar
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

    auto blackboard = config().blackboard;
    if (!blackboard->get("my_goal_x", my_goal_x_)) {
        my_goal_x_ = (robot_id_ == 0) ? -2200.0 : 2200.0; // Default fallback
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
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoalLineDefendNode::onRunning()
{
    double ball_x, ball_y;
    if (!getInput<double>("ball_x", ball_x) || !getInput<double>("ball_y", ball_y)) return BT::NodeStatus::FAILURE;

    // Geometria atualizada por /field_geometry no Blackboard raiz; valores ausentes ou zerados usam o fallback
    double goal_width = DEFAULT_GOAL_WIDTH;
    double area_depth = DEFAULT_AREA_DEPTH;
    double area_width = DEFAULT_AREA_WIDTH;
    if (auto root = config().blackboard ? config().blackboard->rootBlackboard() : nullptr) {
        double value = 0.0;
        if (root->get("goal_width", value) && value > 0.0) goal_width = value;
        if (root->get("penalty_area_depth", value) && value > 0.0) area_depth = value;
        if (root->get("penalty_area_width", value) && value > 0.0) area_width = value;
    }
    const double half   = goal_width / 2.0;
    const double dir    = (my_goal_x_ < 0) ? 1.0 : -1.0;   // +1 se o gol está à esquerda
    const double line_x = my_goal_x_ + dir * GOALKEEPER_LINE_DEPTH;

    // Retângulo dos alvos: o lado do gol é a própria linha do gol (pode entrar no gol, para chutar a bola
    // sobre a linha); nos demais lados o robô inteiro (raio + folga) fica dentro da área
    const double x_far  = my_goal_x_ + dir * (area_depth - ROBOT_RADIUS - kAreaMarginMm);
    const double xmin   = std::min(my_goal_x_, x_far);
    const double xmax   = std::max(my_goal_x_, x_far);
    const double ylim   = area_width / 2.0 - ROBOT_RADIUS - kAreaMarginMm;

    // 1. Posição base: ponto médio da corda bola->postes na borda da área
    double target_x = line_x;
    double target_y = std::clamp(ball_y, -half, half);

    if (dir * (ball_x - my_goal_x_) > 0.0) {   // bola à frente do gol
        auto e1 = entryPoint(ball_x, ball_y, my_goal_x_, -half, xmin, xmax, -ylim, ylim);
        auto e2 = entryPoint(ball_x, ball_y, my_goal_x_,  half, xmin, xmax, -ylim, ylim);
        if (e1 && e2) {
            target_x = 0.5 * (e1->first  + e2->first);
            target_y = 0.5 * (e1->second + e2->second);
        } else {
            // Retas não cruzam a área: segue a bola limitada à área
            target_x = std::clamp(ball_x, xmin, xmax);
            target_y = std::clamp(ball_y, -ylim, ylim);
        }
    }

    // 2. Interceptação (só se o chute for ao gol), com atrito e histerese
    oxebots_interfaces::msg::BallPrediction::SharedPtr pred;
    rclcpp::Time pred_stamp(0, 0, RCL_ROS_TIME);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pred = last_ball_pred_;
        pred_stamp = last_ball_pred_stamp_;
    }

    const rclcpp::Time now = node_->now();
    const bool pred_fresh = pred && (now - pred_stamp).seconds() <= kPredMaxAgeS;

    bool shot = false;
    if (pred_fresh && dir * pred->vx < 0.0 && dir * (pred->x - my_goal_x_) > 0.0) {
        const double min_speed = intercepting_ ? kHoldMinShotSpeed : kMinShotSpeed;
        const double y_margin  = intercepting_ ? kHoldGoalMarginMm : 0.0;
        const double max_time  = intercepting_ ? kMaxImpactTime + kHoldImpactTimeMarginS : kMaxImpactTime;

        if (std::abs(pred->vx) >= min_speed) {
            auto t_goal = timeToReachX(pred->x, pred->vx, pred->ax, my_goal_x_);
            if (t_goal && *t_goal > 0.0 && *t_goal <= max_time) {
                const double y_goal = pred->y + pred->vy * *t_goal + 0.5 * pred->ay * *t_goal * *t_goal;
                if (std::abs(y_goal) <= half + y_margin) {
                    const double t_line = timeToReachX(pred->x, pred->vx, pred->ax, line_x).value_or(0.0);
                    last_intercept_y_ = pred->y + pred->vy * t_line + 0.5 * pred->ay * t_line * t_line;
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
    }

    // 3. Mantém o alvo dentro da área
    target_x = std::clamp(target_x, xmin, xmax);
    target_y = std::clamp(target_y, -ylim, ylim);

    // Orientação: kicker voltado para a bola (visto do ponto alvo). Com a bola muito perto do alvo o rumo
    // é instável, então mantém a orientação anterior (ou olha para o campo se ainda não houver uma).
    double target_w = (my_goal_x_ > 0) ? M_PI : 0.0;
    if (last_target_w_ > -99990.0) target_w = last_target_w_;
    const double ball_dx = ball_x - target_x;
    const double ball_dy = ball_y - target_y;
    if (std::hypot(ball_dx, ball_dy) > kMinFacingDistMm) target_w = std::atan2(ball_dy, ball_dx);

    const bool pos_changed = std::hypot(target_x - last_target_x_, target_y - last_target_y_) > POSITION_UPDATE_THRESHOLD;
    const bool ang_changed = std::abs(std::remainder(target_w - last_target_w_, 2.0 * M_PI)) > ANGLE_UPDATE_THRESHOLD;

    if (pos_changed || ang_changed) {
        auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
        goal_msg->robot_id = robot_id_;
        goal_msg->planner_type = oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE;
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
    }

    return BT::NodeStatus::RUNNING;
}

void GoalLineDefendNode::onHalted() {}

}
