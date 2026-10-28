#include "oxebots_strategy/goal_line_defend_node.h"
#include <cmath>
#include <algorithm>

namespace oxebots_strategy
{

static constexpr double DEFAULT_GOAL_WIDTH = 800.0;
static constexpr double GOALKEEPER_LINE_DEPTH = 300.0; 
static constexpr double MIN_BALL_DIST_TO_LINE = 10.0;
static constexpr double POSITION_UPDATE_THRESHOLD = 5.0;
static constexpr double ROBOT_RADIUS = 90.0;
static constexpr double AREA_DEPTH = 500.0;   
static constexpr double AREA_WIDTH = 1350.0;  
static constexpr double kMinShotSpeed = 500.0;
static constexpr double kMaxImpactTime = 3.0; 

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
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoalLineDefendNode::onRunning()
{
    double ball_x, ball_y;
    if (!getInput<double>("ball_x", ball_x) || !getInput<double>("ball_y", ball_y)) return BT::NodeStatus::FAILURE;

    double goal_width = DEFAULT_GOAL_WIDTH;
    auto blackboard = config().blackboard;
    if (!blackboard->get("goal_width", goal_width)) {
        // Fallback to DEFAULT_GOAL_WIDTH already set
    }
    const double half   = goal_width / 2.0;
    const double dir    = (my_goal_x_ < 0) ? 1.0 : -1.0;   // +1 se o gol está à esquerda
    const double line_x = my_goal_x_ + dir * GOALKEEPER_LINE_DEPTH;

    // Retângulo da área (limites do alvo)
    const double x_far = my_goal_x_ + dir * (AREA_DEPTH - ROBOT_RADIUS);
    const double xmin  = std::min(my_goal_x_, x_far);
    const double xmax  = std::max(my_goal_x_, x_far);
    const double ylim  = AREA_WIDTH / 2.0 - ROBOT_RADIUS;

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

    // 2. Interceptação balística (só se o chute for ao gol)
    oxebots_interfaces::msg::BallPrediction::SharedPtr pred;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pred = last_ball_pred_;
    }

    if (pred && dir * pred->vx < 0.0 && std::abs(pred->vx) >= kMinShotSpeed &&
        dir * (pred->x - my_goal_x_) > 0.0)
    {
        const double t_goal = (my_goal_x_ - pred->x) / pred->vx;   // tempo até o gol
        const double y_goal = pred->y + pred->vy * t_goal;         // y onde cruza o gol
        if (t_goal > 0.0 && t_goal <= kMaxImpactTime && std::abs(y_goal) <= half) {
            const double t_line = std::max(0.0, (line_x - pred->x) / pred->vx);
            target_x = line_x;
            target_y = pred->y + pred->vy * t_line;   // y onde cruza a linha do goleiro
        }
    }

    // 3. Mantém o alvo dentro da área
    target_x = std::clamp(target_x, xmin, xmax);
    target_y = std::clamp(target_y, -ylim, ylim);

    if (std::hypot(target_x - last_target_x_, target_y - last_target_y_) > POSITION_UPDATE_THRESHOLD) {
        auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
        goal_msg->robot_id = robot_id_;
        goal_msg->planner_type = oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE;
        goal_msg->pose.header.stamp = node_ -> now();
        goal_msg->pose.header.frame_id = "map";
        goal_msg->pose.pose.position.x = target_x / 1000.0;
        goal_msg->pose.pose.position.y = target_y / 1000.0;

        double target_w = (my_goal_x_ > 0) ? M_PI : 0.0;
        goal_msg->pose.pose.orientation.z = std::sin(target_w * 0.5);
        goal_msg->pose.pose.orientation.w = std::cos(target_w * 0.5);

        goal_pub_->publish(std::move(goal_msg));
        last_target_y_ = target_y;
        last_target_x_ = target_x;
    }

    return BT::NodeStatus::RUNNING;
}

void GoalLineDefendNode::onHalted() {}

}
