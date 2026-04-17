#include "oxebots_strategy/position_bisector_node.h"


namespace oxebots_strategy
{


PositionBisectorNode::PositionBisectorNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_(node_ptr)
{
if (!getInput<int>("robot_id", robot_id_)) {
   RCLCPP_ERROR(node_->get_logger(), "Erro: porta [robot_id] ausente no XML");
}


auto blackboard = config.blackboard;
if (!blackboard->get("my_goal_x", my_goal_x_)) {
   RCLCPP_ERROR(node_->get_logger(), "Erro: 'my_goal_x' ausente no blackboard");
}


auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
   goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);
   cmd_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", 10);
}


BT::PortsList PositionBisectorNode::providedPorts()
{
   return {
       BT::InputPort<int>("robot_id"),
       BT::InputPort<double>("ball_x"),
       BT::InputPort<double>("ball_y"),
       BT::InputPort<double>("goal_width")
   };
}


BT::NodeStatus PositionBisectorNode::onStart()
{
   RCLCPP_INFO(node_->get_logger(), "Goleiro %d: Iniciando patrulha na bissetriz", robot_id_);
   return BT::NodeStatus::RUNNING;
}


BT::NodeStatus PositionBisectorNode::onRunning()
{
   double ball_x, ball_y, goal_width;


   if (!getInput<double>("ball_x", ball_x) ||
       !getInput<double>("ball_y", ball_y) ||
       !getInput<double>("goal_width", goal_width)) {


       RCLCPP_WARN(node_->get_logger(), "Faltando dados essenciais no Blackboard!");
       return BT::NodeStatus::FAILURE;
   }


   double pl_y = goal_width / 2.0;
   double pr_y = -goal_width / 2.0;


   double pl_x = my_goal_x_;
   double pr_x = my_goal_x_;


   double vl_x, vl_y, vr_x, vr_y;


   vl_x = pl_x - ball_x;
   vl_y = pl_y - ball_y;


   vr_x = pr_x - ball_x;
   vr_y = pr_y - ball_y;


   double mag_vl = std::sqrt((vl_x * vl_x) + (vl_y * vl_y));
   double mag_vr = std::sqrt((vr_x * vr_x) + (vr_y * vr_y));


   double epsilon = 0.001;
   if (mag_vl < epsilon) mag_vl = epsilon;
   if (mag_vr < epsilon) mag_vr = epsilon;


   double bx = (vl_x/mag_vl) + (vr_x/mag_vr);
   double by = (vl_y/mag_vl) + (vr_y/mag_vr);


   if (std::abs(bx) < 0.001) bx = (bx < 0) ? -0.001 : 0.001;


   double t = (my_goal_x_ - ball_x) / bx;


   double target_y = ball_y + (t * by);


   double limit = (goal_width / 2.0) - 90.0;
   target_y = std::clamp(target_y, -limit, limit);


   double target_w = std::atan2(ball_y - target_y, ball_x - my_goal_x_);


   auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
   goal_msg->robot_id = robot_id_;
   goal_msg->pose.header.stamp = node_->now();
   goal_msg->pose.header.frame_id = "map";


   goal_msg->pose.pose.position.x = my_goal_x_ / 1000.0;
   goal_msg->pose.pose.position.y = target_y / 1000.0;




   goal_msg->pose.pose.orientation.x = 0.0;
   goal_msg->pose.pose.orientation.y = 0.0;
   goal_msg->pose.pose.orientation.z = std::sin(target_w * 0.5);
   goal_msg->pose.pose.orientation.w = std::cos(target_w * 0.5);


   goal_pub_->publish(std::move(goal_msg));


   return BT::NodeStatus::RUNNING;
}


void PositionBisectorNode::onHalted()
{
   RCLCPP_INFO(node_->get_logger(), "Goleiro %d: Ação abortada. Acionando freios (0.0).", robot_id_);


   auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
   oxebots_interfaces::msg::RobotCmdData robot_cmd_data;


   robot_cmd_data.id = robot_id_;
   robot_cmd_data.kick_speed = 0.0;
   robot_cmd_data.x_velocity = 0.0;
   robot_cmd_data.y_velocity = 0.0;
   robot_cmd_data.angular_velocity = 0.0;


   cmd_msg->robots.push_back(robot_cmd_data);
   cmd_pub_->publish(std::move(cmd_msg));
}


}
