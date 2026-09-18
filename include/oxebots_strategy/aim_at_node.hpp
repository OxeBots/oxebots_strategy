#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "oxebots_interfaces/msg/robot_motion_override.hpp"
#include "oxebots_interfaces/msg/robot_motion_status.hpp"
#include <cmath>

namespace oxebots_strategy {

class AimAtNode : public BT::StatefulActionNode
{
public:
    AimAtNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node)
    {
        override_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotMotionOverride>("/robot_motion_override", 10);
        status_sub_ = node_->create_subscription<oxebots_interfaces::msg::RobotMotionStatus>(
            "/robot_motion_status", 10, [this](const oxebots_interfaces::msg::RobotMotionStatus::SharedPtr msg) {
                if (msg->robot_id == robot_id_) last_status_ = msg;
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<uint32_t>("robot_id"),
            BT::InputPort<double>("target_x"),
            BT::InputPort<double>("target_y")
        };
    }

    BT::NodeStatus onStart() override {
        if (!getInput("robot_id", robot_id_)) return BT::NodeStatus::FAILURE;
        return step();
    }

    BT::NodeStatus onRunning() override {
        return step();
    }

    void onHalted() override {
        auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
        msg->robot_id = robot_id_;
        msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_NONE;
        override_pub_->publish(std::move(msg));
    }

private:
    BT::NodeStatus step() {
        double target_x, target_y, rx, ry;

        if (!getInput("target_x", target_x) || !getInput("target_y", target_y)) {
            return BT::NodeStatus::FAILURE;
        }

        // Se o controlador disser que está alinhado, libera o comando e avança a árvore
        if (last_status_ && last_status_->active_mode == oxebots_interfaces::msg::RobotMotionOverride::MODE_ALIGN_TO_ANGLE && last_status_->aligned) {
            onHalted();
            return BT::NodeStatus::SUCCESS;
        }

        if (!config().blackboard->get("robot_x", rx) || !config().blackboard->get("robot_y", ry)) {
            return BT::NodeStatus::FAILURE;
        }

        double angle = std::atan2(target_y - ry, target_x - rx);

        auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
        msg->robot_id = robot_id_;
        msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_ALIGN_TO_ANGLE;
        msg->target_angle = angle;
        msg->angle_threshold_deg = 5.0; // Precisão de 5 graus
        msg->max_angular_speed = 4.0;
        msg->p_gain = 4.0;
        
        override_pub_->publish(std::move(msg));

        return BT::NodeStatus::RUNNING;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<oxebots_interfaces::msg::RobotMotionOverride>::SharedPtr override_pub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotMotionStatus>::SharedPtr status_sub_;
    oxebots_interfaces::msg::RobotMotionStatus::SharedPtr last_status_;
    uint32_t robot_id_ = 0;
};

} // namespace oxebots_strategy