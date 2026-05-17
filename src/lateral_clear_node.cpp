#include "oxebots_strategy/lateral_clear_node.h"
#include <cmath>
#include <memory>

namespace oxebots_strategy
{

static constexpr double LATERAL_TARGET_Y = 2000.0;
static constexpr double KICK_SPEED_DEFAULT = 3.0f;
static constexpr double POS_UPDATE_THRESHOLD = 10.0;
static constexpr double ANG_UPDATE_THRESHOLD = 0.05;

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

    double target_x = ball_x;
    double target_y = ball_y;
    double target_side_y = (ball_y > 0) ? LATERAL_TARGET_Y : -LATERAL_TARGET_Y;
    double target_w = std::atan2(target_side_y - ball_y, 0.0 - ball_x);

    bool target_changed = (std::abs(target_x - last_target_x_) > POS_UPDATE_THRESHOLD || 
                           std::abs(target_y - last_target_y_) > POS_UPDATE_THRESHOLD ||
                           std::abs(target_w - last_target_w_) > ANG_UPDATE_THRESHOLD);

    if (target_changed) {
        auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
        goal_msg->robot_id = robot_id_;
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

    // Check if we arrived at the ball to trigger SUCCESS
    double dx = target_x - rx;
    double dy = target_y - ry;
    double dist = std::sqrt(dx*dx + dy*dy);
    
    // Normalize angle difference
    double angle_diff = target_w - ryaw;
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    if (dist < 150.0 && std::abs(angle_diff) < 0.2) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void LateralClearNode::onHalted() {}

}
