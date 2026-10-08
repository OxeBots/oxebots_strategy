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

BT::PortsList GoalLineDefendNode::providedPorts()
{
    return { 
        BT::InputPort<uint32_t>("robot_id"),
        BT::InputPort<double>("ball_y")
    };
}

void GoalLineDefendNode::ballPredictionCallback(const oxebots_interfaces::msg::BallPrediction::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_ball_pred_ = msg;
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

    double limit = goal_width / 2.0;
    double target_y = std::clamp(ball_y, -limit, limit);

    double offset_x = (my_goal_x_ > 0) ? -GOAL_LINE_OFFSET : GOAL_LINE_OFFSET;
    double line_x = my_goal_x_ + offset_x;

    // Copia a predição da bola recebida pelo Kalman Filter
    oxebots_interfaces::msg::BallPrediction::SharedPtr pred;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pred = last_ball_pred_;
    }

    if (pred) {
        double bx = pred->x;
        double by = pred->y;
        double b_vx = pred->vx;
        double b_vy = pred->vy;

        constexpr double MIN_SHOT_SPEED = 200.0; // mm/s
        bool moving_towards_goal = false;
        if (my_goal_x_ < 0 && b_vx < -MIN_SHOT_SPEED && bx > line_x) {
            moving_towards_goal = true;
        } else if (my_goal_x_ > 0 && b_vx > MIN_SHOT_SPEED && bx < line_x) {
            moving_towards_goal = true;
        }

        if (moving_towards_goal) {
            double t = (line_x - bx) / b_vx;
            // Se a bola vai cruzar a linha nos próximos 3 segundos
            if (t > 0.0 && t <= 3.0) {
                double pred_y = by + b_vy * t;
                target_y = std::clamp(pred_y, -limit, limit);
            }
        }
    }

    if (std::abs(target_y - last_target_y_) > POSITION_UPDATE_THRESHOLD) {
        auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
        goal_msg->robot_id = robot_id_;
        goal_msg->pose.header.stamp = node_->now();
        goal_msg->pose.header.frame_id = "map";
        
        goal_msg->pose.pose.position.x = line_x / 1000.0;
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