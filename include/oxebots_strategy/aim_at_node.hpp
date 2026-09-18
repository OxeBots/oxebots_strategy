#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "oxebots_interfaces/msg/robot_motion_override.hpp"
#include <cmath>

namespace oxebots_strategy {

class AimAtNode : public BT::SyncActionNode
{
public:
    AimAtNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        override_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotMotionOverride>("/robot_motion_override", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<uint32_t>("robot_id"),
            BT::InputPort<double>("target_x"),
            BT::InputPort<double>("target_y")
        };
    }

    BT::NodeStatus tick() override
    {
        uint32_t robot_id;
        double target_x, target_y, rx, ry;

        if (!getInput("robot_id", robot_id) || !getInput("target_x", target_x) || !getInput("target_y", target_y)) {
            return BT::NodeStatus::FAILURE;
        }

        // Lê a posição atual do robô do Blackboard
        if (!config().blackboard->get("robot_x", rx) || !config().blackboard->get("robot_y", ry)) {
            return BT::NodeStatus::FAILURE;
        }

        // Calcula o ângulo em radianos apontando para o alvo
        double angle = std::atan2(target_y - ry, target_x - rx);

        auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
        msg->robot_id = robot_id;
        msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_ALIGN_TO_ANGLE;
        msg->target_angle = angle;
        
        override_pub_->publish(std::move(msg));

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<oxebots_interfaces::msg::RobotMotionOverride>::SharedPtr override_pub_;
};

} // namespace oxebots_strategy