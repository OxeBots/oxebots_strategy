#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <memory>
#include <string>
#include "oxebots_interfaces/msg/ball_prediction.hpp"
#include "oxebots_interfaces/msg/robot_prediction.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"

namespace oxebots_strategy {

class AiParser : public BT::StatefulActionNode
{
public:
    AiParser(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void ball_callback(const oxebots_interfaces::msg::BallPrediction::SharedPtr msg);
    void robot_callback(const oxebots_interfaces::msg::RobotPrediction::SharedPtr msg);

    bool prepare_input_tensor(unsigned int robot_id);
    void publish_action(unsigned int robot_id, const float* action_data);

    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<oxebots_interfaces::msg::BallPrediction>::SharedPtr ball_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotPrediction>::SharedPtr robot_sub_;
    rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_pub_;

    oxebots_interfaces::msg::BallPrediction::SharedPtr last_ball_state_;
    oxebots_interfaces::msg::RobotPrediction::SharedPtr last_robot_state_;

    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session_;
    std::vector<float> input_tensor_values_;
    std::vector<int64_t> input_shape_ = {1, 35};
};

}