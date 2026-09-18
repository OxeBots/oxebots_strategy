#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include <cmath>

namespace oxebots_strategy {

class InterceptBallNode : public BT::SyncActionNode
{
public:
    InterceptBallNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<uint32_t>("robot_id"),
            BT::InputPort<double>("face_x"),
            BT::InputPort<double>("face_y")
        };
    }

    BT::NodeStatus tick() override
    {
        uint32_t robot_id;
        double face_x, face_y;
        double ball_x, ball_y, ball_vx, ball_vy;

        if (!getInput("robot_id", robot_id) || !getInput("face_x", face_x) || !getInput("face_y", face_y)) {
            return BT::NodeStatus::FAILURE;
        }

        if (!config().blackboard->get("ball_x", ball_x) || !config().blackboard->get("ball_y", ball_y) ||
            !config().blackboard->get("ball_vx", ball_vx) || !config().blackboard->get("ball_vy", ball_vy)) {
            return BT::NodeStatus::FAILURE;
        }

        // Predição super simples: onde a bola estará em 0.5 segundos?
        // (Você pode evoluir isso depois integrando com seu d_star_planner)
        double lookahead_time = 0.5; 
        double intercept_x = ball_x + (ball_vx * lookahead_time);
        double intercept_y = ball_y + (ball_vy * lookahead_time);

        // Calcula orientação final
        double target_yaw = std::atan2(face_y - intercept_y, face_x - intercept_x);

        // Monta o nav_msgs::msg::Pose e envia
        auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
        goal_msg->robot_id = robot_id;
        goal_msg->planner_type = oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE;
        
        goal_msg->pose.pose.position.x = intercept_x / 1000.0; // Assume que o Planner usa metros
        goal_msg->pose.pose.position.y = intercept_y / 1000.0;
        
        // Conversão de Yaw para Quaternion simplificada (eixo Z)
        goal_msg->pose.pose.orientation.w = std::cos(target_yaw / 2.0);
        goal_msg->pose.pose.orientation.z = std::sin(target_yaw / 2.0);

        goal_pub_->publish(std::move(goal_msg));

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub_;
};

} // namespace oxebots_strategy