#include "oxebots_strategy/defender_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

constexpr double kMillimetersToMeters = 0.001;
constexpr double kGoalCenterY = 0.0;

double computeHeading(const Pose2D& from, const Pose2D& to)
{
    return std::atan2(to.y - from.y, to.x - from.x);
}

Pose2D lerpTowardGoal(const Pose2D& ball, double my_goal_x, double alpha)
{
    Pose2D target{};
    target.x = ball.x + alpha * (my_goal_x - ball.x);
    target.y = ball.y + alpha * (kGoalCenterY - ball.y);
    target.theta = computeHeading(target, ball);
    return target;
}

struct DefenderControllerContext
{
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub;
    uint32_t robot_id{1};
};

DefenderControllerContext& getControllerContext()
{
    static DefenderControllerContext context;
    return context;
}

}  // namespace

void configureDefenderController(const rclcpp::Node::SharedPtr& node, uint32_t robot_id)
{
    auto& context = getControllerContext();
    auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    context.node = node;
    context.goal_pub = node->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);
    context.robot_id = robot_id;
}

void sendToController(const Pose2D& target)
{
    auto& context = getControllerContext();
    if (!context.goal_pub || !context.node) {
        return;
    }

    auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
    goal_msg->robot_id = context.robot_id;
    goal_msg->pose.header.stamp = context.node->now();
    goal_msg->pose.header.frame_id = "map";
    goal_msg->pose.pose.position.x = target.x * kMillimetersToMeters;
    goal_msg->pose.pose.position.y = target.y * kMillimetersToMeters;
    goal_msg->pose.pose.orientation.x = 0.0;
    goal_msg->pose.pose.orientation.y = 0.0;
    goal_msg->pose.pose.orientation.z = std::sin(target.theta * 0.5);
    goal_msg->pose.pose.orientation.w = std::cos(target.theta * 0.5);

    context.goal_pub->publish(std::move(goal_msg));
}

// ---------------- CONDITIONS ----------------

BT::PortsList IsBallInOpponentField::providedPorts()
{
    return {
        BT::InputPort<double>("ball_x"),
        BT::InputPort<bool>("is_yellow")
    };
}

BT::NodeStatus IsBallInOpponentField::tick()
{
    double ball_x = 0.0;
    bool is_yellow = false;

    if (!getInput("ball_x", ball_x) || !getInput("is_yellow", is_yellow)) {
        return BT::NodeStatus::FAILURE;
    }

    const bool in_opponent_field = is_yellow ? (ball_x < 0.0) : (ball_x > 0.0);
    return in_opponent_field ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsOpponentControllingBall::providedPorts()
{
    return {BT::InputPort<bool>("opponent_controlling")};
}

BT::NodeStatus IsOpponentControllingBall::tick()
{
    bool opponent_controlling = false;
    if (!getInput("opponent_controlling", opponent_controlling)) {
        return BT::NodeStatus::FAILURE;
    }
    return opponent_controlling ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsBallNearGoal::providedPorts()
{
    return {
        BT::InputPort<double>("ball_x"),
        BT::InputPort<double>("ball_y"),
        BT::InputPort<double>("my_goal_x"),
        BT::InputPort<double>("goal_x_threshold", 700.0, "Distance to own goal line in mm"),
        BT::InputPort<double>("goal_band_half_width", 900.0, "Half width of danger band in mm")
    };
}

BT::NodeStatus IsBallNearGoal::tick()
{
    double ball_x = 0.0;
    double ball_y = 0.0;
    double my_goal_x = 0.0;
    double goal_x_threshold = 700.0;
    double goal_band_half_width = 900.0;

    if (!getInput("ball_x", ball_x) || !getInput("ball_y", ball_y) || !getInput("my_goal_x", my_goal_x)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("goal_x_threshold", goal_x_threshold);
    getInput("goal_band_half_width", goal_band_half_width);

    const bool near_goal_line = std::abs(ball_x - my_goal_x) <= goal_x_threshold;
    const bool inside_goal_band = std::abs(ball_y) <= goal_band_half_width;

    return (near_goal_line && inside_goal_band) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsDefenderCloserThanAttacker::providedPorts()
{
    return {
        BT::InputPort<double>("defender_ball_dist"),
        BT::InputPort<double>("attacker_ball_dist"),
        BT::InputPort<double>("margin", 0.0, "Extra margin in mm")
    };
}

BT::NodeStatus IsDefenderCloserThanAttacker::tick()
{
    double defender_ball_dist = std::numeric_limits<double>::infinity();
    double attacker_ball_dist = std::numeric_limits<double>::infinity();
    double margin = 0.0;

    if (!getInput("defender_ball_dist", defender_ball_dist) || !getInput("attacker_ball_dist", attacker_ball_dist)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("margin", margin);

    if (!std::isfinite(defender_ball_dist) || !std::isfinite(attacker_ball_dist)) {
        return BT::NodeStatus::FAILURE;
    }

    const bool defender_is_closer = (defender_ball_dist + margin) <= attacker_ball_dist;
    return defender_is_closer ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsBallMovingTowardsGoal::providedPorts()
{
    return {
        BT::InputPort<double>("ball_vx"),
        BT::InputPort<double>("my_goal_x"),
        BT::InputPort<double>("velocity_threshold", 120.0, "Minimum vx magnitude in mm/s")
    };
}

BT::NodeStatus IsBallMovingTowardsGoal::tick()
{
    double ball_vx = 0.0;
    double my_goal_x = 0.0;
    double velocity_threshold = 120.0;

    if (!getInput("ball_vx", ball_vx) || !getInput("my_goal_x", my_goal_x)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("velocity_threshold", velocity_threshold);

    const bool moving_toward_goal = (my_goal_x < 0.0) ? (ball_vx < -velocity_threshold)
                                                      : (ball_vx > velocity_threshold);
    return moving_toward_goal ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------- ACTIONS ----------------

BT::PortsList GoToSafe::providedPorts()
{
    return {
        BT::InputPort<double>("my_goal_x"),
        BT::InputPort<Pose2D>("ball"),
        BT::InputPort<double>("safe_offset_x", 400.0, "Offset from own goal line in mm")
    };
}

BT::NodeStatus GoToSafe::tick()
{
    double my_goal_x = 0.0;
    double safe_offset_x = 400.0;
    Pose2D ball{};

    if (!getInput("my_goal_x", my_goal_x) || !getInput("ball", ball)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("safe_offset_x", safe_offset_x);

    Pose2D target{};
    target.x = my_goal_x - std::copysign(safe_offset_x, my_goal_x);
    target.y = std::clamp(ball.y, -700.0, 700.0);
    target.theta = computeHeading(target, ball);

    sendToController(target);
    return BT::NodeStatus::SUCCESS;
}

// ---------------- BLOCK SHOT ----------------

BT::PortsList BlockShot::providedPorts()
{
    return {
        BT::InputPort<Pose2D>("ball"),
        BT::InputPort<double>("my_goal_x"),
        BT::InputPort<double>("alpha", 0.75, "Interpolation factor from ball to own goal")
    };
}

BT::NodeStatus BlockShot::tick()
{
    Pose2D ball{};
    double my_goal_x = 0.0;
    double alpha = 0.75;

    if (!getInput("ball", ball) || !getInput("my_goal_x", my_goal_x)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("alpha", alpha);

    Pose2D target = lerpTowardGoal(ball, my_goal_x, alpha);
    sendToController(target);
    return BT::NodeStatus::SUCCESS;
}

// ---------------- PRESSURE ----------------

BT::PortsList PressureBall::providedPorts()
{
    return {
        BT::InputPort<Pose2D>("ball"),
        BT::InputPort<double>("my_goal_x"),
        BT::InputPort<double>("pressure_offset", 120.0, "Behind-ball offset in mm")
    };
}

BT::NodeStatus PressureBall::tick()
{
    Pose2D ball{};
    double my_goal_x = 0.0;
    double pressure_offset = 120.0;

    if (!getInput("ball", ball) || !getInput("my_goal_x", my_goal_x)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("pressure_offset", pressure_offset);

    const double direction_to_goal = (my_goal_x >= ball.x) ? 1.0 : -1.0;

    Pose2D target{};
    target.x = ball.x + direction_to_goal * pressure_offset;
    target.y = ball.y;
    target.theta = computeHeading(target, ball);

    sendToController(target);
    return BT::NodeStatus::SUCCESS;
}

// ---------------- BLOCK ANGLE ----------------

BT::PortsList BlockAngle::providedPorts()
{
    return {
        BT::InputPort<Pose2D>("ball"),
        BT::InputPort<double>("my_goal_x"),
        BT::InputPort<double>("alpha", 0.6, "Interpolation factor from ball to own goal")
    };
}

BT::NodeStatus BlockAngle::tick()
{
    Pose2D ball{};
    double my_goal_x = 0.0;
    double alpha = 0.6;

    if (!getInput("ball", ball) || !getInput("my_goal_x", my_goal_x)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("alpha", alpha);

    Pose2D target = lerpTowardGoal(ball, my_goal_x, alpha);
    sendToController(target);
    return BT::NodeStatus::SUCCESS;
}

// ---------------- INTERCEPT ----------------

BT::PortsList InterceptBall::providedPorts()
{
    return {
        BT::InputPort<Pose2D>("ball"),
        BT::InputPort<Vector2D>("ball_vel"),
        BT::InputPort<double>("intercept_time", 0.45, "Time horizon in seconds")
    };
}

BT::NodeStatus InterceptBall::tick()
{
    Pose2D ball{};
    Vector2D vel{};
    double intercept_time = 0.45;

    if (!getInput("ball", ball) || !getInput("ball_vel", vel)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("intercept_time", intercept_time);

    Pose2D target{};
    target.x = std::clamp(ball.x + vel.x * intercept_time, -2200.0, 2200.0);
    target.y = std::clamp(ball.y + vel.y * intercept_time, -1500.0, 1500.0);
    target.theta = computeHeading(target, ball);

    sendToController(target);
    return BT::NodeStatus::SUCCESS;
}

// ---------------- DEFENSIVE ----------------

BT::PortsList DefensivePosition::providedPorts()
{
    return {
        BT::InputPort<Pose2D>("ball"),
        BT::InputPort<double>("my_goal_x"),
        BT::InputPort<double>("alpha", 0.6, "Interpolation factor from ball to own goal")
    };
}

BT::NodeStatus DefensivePosition::tick()
{
    Pose2D ball{};
    double my_goal_x = 0.0;
    double alpha = 0.6;

    if (!getInput("ball", ball) || !getInput("my_goal_x", my_goal_x)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("alpha", alpha);

    Pose2D target = lerpTowardGoal(ball, my_goal_x, alpha);
    target.y = std::clamp(target.y, -900.0, 900.0);

    sendToController(target);
    return BT::NodeStatus::SUCCESS;
}