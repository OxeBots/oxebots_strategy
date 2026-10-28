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
    const double half = goal_width/2.0;
    const double dir = (my_goal_x_ < 0) ? 1.0 : -1.0;
    const double line_x = my_goal_x_ + dir * GOALKEEPER_LINE_DEPTH;
    double target_y = ball_y;
    const double ball_dist_to_line = dir * (ball_x - line_x);
    const bool ball_in_front = dir * (ball_x - my_goal_x_) > 0.0;
    if (ball_dist_to_line > MIN_BALL_DIST_TO_LINE && ball_in_front) {
        const double ux = my_goal_x_ - ball_x;
        const double n1 = std::hypot(ux, -half - ball_y);
        const double n2 = std::hypot(ux,  half - ball_y);
        const double bx = ux / n1 + ux / n2;
        const double by = (-half - ball_y) / n1 + (half - ball_y) / n2;
        target_y = ball_y + ((line_x - ball_x) / bx) * by;
    }
    // Interceptação balística: quando o adversário chuta, prevê onde a bola cruzará a linha do gol
    oxebots_interfaces::msg::BallPrediction::SharedPtr pred;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pred = last_ball_pred_;
    }

     if (pred) {
        const double dx_to_line = line_x - pred->x;
        constexpr double kMinShotSpeed = 250.0;
        if ((dx_to_line * pred->vx > 0.0) && std::abs(pred->vx) >= kMinShotSpeed) {
            const double t_impact = dx_to_line / pred->vx;
            if (t_impact > 0.0 && t_impact <= 3.0) {
                target_y = pred->y + pred->vy * t_impact; // onde a bola cruza a LINHA DO GOLEIRO
            }
        }
    }

       // ~90 mm na SSL
    target_y = std::clamp(target_y, -half, half);
    const double target_x = line_x;
    if (std::abs(target_y - last_target_y_) > POSITION_UPDATE_THRESHOLD) {
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
    }

    return BT::NodeStatus::RUNNING;
}

void GoalLineDefendNode::onHalted() {}

}
