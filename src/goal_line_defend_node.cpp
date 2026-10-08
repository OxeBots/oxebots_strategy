#include "oxebots_strategy/goal_line_defend_node.h"
#include <cmath>
#include <algorithm>

namespace oxebots_strategy
{

static constexpr double DEFAULT_GOAL_WIDTH = 1000.0;
static constexpr double GOAL_LINE_OFFSET = 100.0;
static constexpr double POSITION_UPDATE_THRESHOLD = 5.0;

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
    double ball_y;
    if (!getInput<double>("ball_y", ball_y)) return BT::NodeStatus::FAILURE;

    double goal_width = DEFAULT_GOAL_WIDTH;
    auto blackboard = config().blackboard;
    if (!blackboard->get("goal_width", goal_width)) {
        // Fallback to DEFAULT_GOAL_WIDTH already set
    }

    double target_y = ball_y;

    // Interceptação balística: quando o adversário chuta, prevê onde a bola cruzará a linha do gol
    oxebots_interfaces::msg::BallPrediction::SharedPtr pred;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pred = last_ball_pred_;
    }

    if (pred) {
        double bx = pred->x;
        double by = pred->y;
        double vx = pred->vx;
        double vy = pred->vy;

        double dx_to_goal = my_goal_x_ - bx;
        // O vetor aponta para o nosso gol se dx_to_goal e vx tiverem o mesmo sinal
        constexpr double kMinShotSpeed = 250.0; // mm/s (0.25 m/s) para ignorar ruído
        if ((dx_to_goal * vx > 0.0) && std::abs(vx) >= kMinShotSpeed) {
            double t_impact = dx_to_goal / vx;
            if (t_impact > 0.0 && t_impact <= 3.0) {
                target_y = by + vy * t_impact;
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                    "Goleiro interceptando chute: t_impact=%.2fs, y_intercept=%.1f (ball_y=%.1f)",
                    t_impact, target_y, ball_y);
            }
        }
    }

    double limit = goal_width / 2.0;
    target_y = std::clamp(target_y, -limit, limit);

    if (std::abs(target_y - last_target_y_) > POSITION_UPDATE_THRESHOLD) {
        auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
        goal_msg->robot_id = robot_id_;
        goal_msg->pose.header.stamp = node_->now();
        goal_msg->pose.header.frame_id = "map";
        
        double offset_x = (my_goal_x_ > 0) ? -GOAL_LINE_OFFSET : GOAL_LINE_OFFSET;
        goal_msg->pose.pose.position.x = (my_goal_x_ + offset_x) / 1000.0;
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
