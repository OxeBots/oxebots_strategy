#include "oxebots_strategy/position_bisector_node.h"
#include <cmath>
#include <algorithm>

namespace oxebots_strategy
{

static constexpr double DIST_FAR_FIELD = 3000.0;
static constexpr double DIST_MID_FIELD = 1500.0;
static constexpr double OFFSET_FAR = 400.0;
static constexpr double OFFSET_MID = 150.0;
static constexpr double WEIGHT_BISECTOR = 0.8;
static constexpr double WEIGHT_BALL = 0.2;
static constexpr double GOAL_EDGE_SAFETY = 90.0;
static constexpr double POS_UPDATE_THRESHOLD = 5.0;
static constexpr double ANG_UPDATE_THRESHOLD = 0.05;

PositionBisectorNode::PositionBisectorNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_(node_ptr)
{
    auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);
}

BT::PortsList PositionBisectorNode::providedPorts()
{
   return {
       BT::InputPort<unsigned int>("robot_id"),
       BT::InputPort<double>("ball_x"),
       BT::InputPort<double>("ball_y"),
       BT::InputPort<double>("goal_width")
   };
}

BT::NodeStatus PositionBisectorNode::onStart()
{
   if (!getInput<unsigned int>("robot_id", robot_id_)) {
       robot_id_ = 0;
   }
   
   last_target_x_ = -99999.0;
   last_target_y_ = -99999.0;
   last_target_w_ = -99999.0;

   return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PositionBisectorNode::onRunning()
{
   double ball_x, ball_y, goal_width;

   if (!getInput<double>("ball_x", ball_x) || !getInput<double>("ball_y", ball_y)) return BT::NodeStatus::FAILURE;
   if (!getInput<double>("goal_width", goal_width)) return BT::NodeStatus::FAILURE;
   if (!config().blackboard->get("my_goal_x", my_goal_x_)) return BT::NodeStatus::FAILURE;

   double dist_to_ball = std::abs(ball_x - my_goal_x_);
   double target_x = my_goal_x_;
   
   if (dist_to_ball > DIST_FAR_FIELD) {
       target_x = my_goal_x_ + (my_goal_x_ > 0 ? -OFFSET_FAR : OFFSET_FAR);
   } else if (dist_to_ball > DIST_MID_FIELD) {
       target_x = my_goal_x_ + (my_goal_x_ > 0 ? -OFFSET_MID : OFFSET_MID);
   }

   double pl_y = goal_width / 2.0;
   double pr_y = -goal_width / 2.0;
   double vl_x = my_goal_x_ - ball_x;
   double vl_y = pl_y - ball_y;
   double vr_x = my_goal_x_ - ball_x;
   double vr_y = pr_y - ball_y;
   double mag_vl = std::hypot(vl_x, vl_y);
   double mag_vr = std::hypot(vr_x, vr_y);

   double bx = (vl_x / std::max(mag_vl, 0.001)) + (vr_x / std::max(mag_vr, 0.001));
   double by = (vl_y / std::max(mag_vl, 0.001)) + (vr_y / std::max(mag_vr, 0.001));
   double t = (target_x - ball_x) / std::max(std::abs(bx), 0.001);
   double target_y_bisector = ball_y + (t * by);

   double target_y = (target_y_bisector * WEIGHT_BISECTOR) + (ball_y * WEIGHT_BALL);
   double limit = (goal_width / 2.0) - GOAL_EDGE_SAFETY;
   target_y = std::clamp(target_y, -limit, limit);

   double target_w = std::atan2(ball_y - target_y, ball_x - target_x);

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

   return BT::NodeStatus::RUNNING;
}

void PositionBisectorNode::onHalted() {}

}
