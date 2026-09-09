#include <ament_index_cpp/get_package_share_directory.hpp>
#include "oxebots_strategy/ai_parse.hpp"
#include <cmath>
#include <algorithm>

namespace oxebots_strategy {

AiParser::AiParser(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node), env(ORT_LOGGING_LEVEL_WARNING, "OxeBots_AIParser")
{
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    std::string model_name;
    if (!getInput<std::string>("model_name", model_name)) {
        model_name = "attacker_treino1_best.onnx"; // Fallback
    }

    std::string pkg_path = ament_index_cpp::get_package_share_directory("oxebots_strategy");
    std::string model_path_str = pkg_path + "/models/" + model_name;

#ifdef _WIN32
    std::wstring w_model_path(model_path_str.begin(), model_path_str.end());
    session_ = std::make_unique<Ort::Session>(env, w_model_path.c_str(), session_options);
#else
    session_ = std::make_unique<Ort::Session>(env, model_path_str.c_str(), session_options);
#endif

    input_tensor_values_.assign(35, 0.0f);

    auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    cmd_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", cmd_qos);

    ball_sub_ = node_->create_subscription<oxebots_interfaces::msg::BallPrediction>(
        "/ball_predicted", 10, std::bind(&AiParser::ball_callback, this, std::placeholders::_1));

    robot_sub_ = node_->create_subscription<oxebots_interfaces::msg::RobotPrediction>(
        "/robot_predicted", 10, std::bind(&AiParser::robot_callback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "AiParser loading model: %s", model_name.c_str());
}

BT::PortsList AiParser::providedPorts()
{
    return {
        BT::InputPort<unsigned int>("robot_id"),
        BT::InputPort<std::string>("model_name", "Model name (.onnx) on /models directory")
    };
}

void AiParser::ball_callback(const oxebots_interfaces::msg::BallPrediction::SharedPtr msg)
{
    last_ball_state_ = msg;
}

void AiParser::robot_callback(const oxebots_interfaces::msg::RobotPrediction::SharedPtr msg)
{
    last_robot_state_ = msg;
}

BT::NodeStatus AiParser::onStart()
{
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AiParser::onRunning()
{
    if (!last_ball_state_ || !last_robot_state_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "AI parser waiting for data");
        return BT::NodeStatus::RUNNING;
    }

    unsigned int robot_id;
    if (!getInput<unsigned int>("robot_id", robot_id)) {
        RCLCPP_ERROR(node_->get_logger(), "Missing robot_id");
        return BT::NodeStatus::FAILURE;
    }

    if (!prepare_input_tensor(robot_id)) {
        return BT::NodeStatus::RUNNING;
    }

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_tensor_values_.data(), input_tensor_values_.size(), input_shape_.data(), input_shape_.size());

    const char* input_names[] = {"observation"};
    const char* output_names[] = {"action"};

    auto output_tensors = session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

    float* action_data = output_tensors.front().GetTensorMutableData<float>();

    publish_action(robot_id, action_data);

    return BT::NodeStatus::RUNNING;
}

bool AiParser::prepare_input_tensor(unsigned int robot_id)
{
    const oxebots_interfaces::msg::RobotPredictionData* me = nullptr;
    const oxebots_interfaces::msg::RobotPredictionData* partner = nullptr;
    const oxebots_interfaces::msg::RobotPredictionData* support = nullptr;

    // let id 1 and 2 for attk
    unsigned int partner_id = (robot_id == 1) ? 2 : 1;

    for (const auto& ally : last_robot_state_->allies) {
        if (ally.id == robot_id) me = &ally;
        else if (ally.id == partner_id) partner = &ally;
        else support = &ally;
    }

    if (!me) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Robot (ID %u) not detected!", robot_id);
        return false;
    }

    const float MAX_POS = 2.75f;
    const float MAX_VEL = 1.5f;
    const float MAX_ANG_VEL = 5.0f;

    auto norm_clamp = [](double val, double max_val) {
        return std::clamp(static_cast<float>(val / max_val), -1.2f, 1.2f);
    };

    double bx = last_ball_state_->x / 1000.0;
    double by = last_ball_state_->y / 1000.0;
    double bvx = last_ball_state_->vx / 1000.0;
    double bvy = last_ball_state_->vy / 1000.0;

    double mx = me->x / 1000.0;
    double my = me->y / 1000.0;
    double mtheta = me->orientation;

    double dx_ball = bx - mx;
    double dy_ball = by - my;
    double dist_ball = std::hypot(dx_ball, dy_ball);
    double angle_to_ball = std::atan2(dy_ball, dx_ball);
    double angle_diff = std::fmod(angle_to_ball - mtheta + M_PI, 2 * M_PI) - M_PI;

    // [0:8] Próprio Robô
    input_tensor_values_[0] = norm_clamp(mx, MAX_POS);
    input_tensor_values_[1] = norm_clamp(my, MAX_POS);
    input_tensor_values_[2] = static_cast<float>(std::sin(mtheta));
    input_tensor_values_[3] = static_cast<float>(std::cos(mtheta));
    input_tensor_values_[4] = norm_clamp(me->vx / 1000.0, MAX_VEL);
    input_tensor_values_[5] = norm_clamp(me->vy / 1000.0, MAX_VEL);
    input_tensor_values_[6] = norm_clamp(me->vorientation, MAX_ANG_VEL);
    input_tensor_values_[7] = (dist_ball <= 0.125 && std::abs(angle_diff) < (25.0 * M_PI / 180.0)) ? 1.0f : 0.0f; // Infrared

    // [8:14] Bola
    input_tensor_values_[8] = norm_clamp(dx_ball, MAX_POS);
    input_tensor_values_[9] = norm_clamp(dy_ball, MAX_POS);
    input_tensor_values_[10] = norm_clamp(bvx - (me->vx / 1000.0), MAX_VEL);
    input_tensor_values_[11] = norm_clamp(bvy - (me->vy / 1000.0), MAX_VEL);
    input_tensor_values_[12] = norm_clamp(dist_ball, MAX_POS * 2.0);
    input_tensor_values_[13] = static_cast<float>(angle_diff / M_PI);

    // [14:22] Parceiro
    if (partner) {
        input_tensor_values_[14] = norm_clamp((partner->x / 1000.0) - mx, MAX_POS);
        input_tensor_values_[15] = norm_clamp((partner->y / 1000.0) - my, MAX_POS);
        input_tensor_values_[16] = static_cast<float>(std::sin(partner->orientation));
        input_tensor_values_[17] = static_cast<float>(std::cos(partner->orientation));
        input_tensor_values_[18] = norm_clamp(partner->vx / 1000.0, MAX_VEL);
        input_tensor_values_[19] = norm_clamp(partner->vy / 1000.0, MAX_VEL);
        input_tensor_values_[20] = norm_clamp(partner->vorientation, MAX_ANG_VEL);
        input_tensor_values_[21] = 0.0f;
    } else {
        for (int i = 14; i < 22; i++) input_tensor_values_[i] = 0.0f;
    }

    // [22:24] Apoio
    if (support) {
        input_tensor_values_[22] = norm_clamp((support->x / 1000.0) - mx, MAX_POS);
        input_tensor_values_[23] = norm_clamp((support->y / 1000.0) - my, MAX_POS);
    } else {
        input_tensor_values_[22] = 0.0f; input_tensor_values_[23] = 0.0f;
    }

    // [24:30] Inimigos
    for (int i = 0; i < 3; i++) {
        if (i < static_cast<int>(last_robot_state_->enemies.size())) {
            const auto& enemy = last_robot_state_->enemies[i];
            input_tensor_values_[24 + (i * 2)] = norm_clamp((enemy.x / 1000.0) - mx, MAX_POS);
            input_tensor_values_[25 + (i * 2)] = norm_clamp((enemy.y / 1000.0) - my, MAX_POS);
        } else {
            input_tensor_values_[24 + (i * 2)] = 0.0f;
            input_tensor_values_[25 + (i * 2)] = 0.0f;
        }
    }

    // [30:32] One-Hot ID
    input_tensor_values_[30] = (robot_id == 1) ? 1.0f : 0.0f;
    input_tensor_values_[31] = (robot_id == 1) ? 0.0f : 1.0f;

    // [32:35] Contexto Tático
    input_tensor_values_[32] = (dist_ball < 0.18 || input_tensor_values_[7] == 1.0f) ? 1.0f : 0.0f;
    input_tensor_values_[33] = 0.0f;

    double ball_speed = std::hypot(bvx, bvy);
    bool ball_towards_me = (ball_speed > 0.6 && std::abs(angle_to_ball + M_PI - std::atan2(bvy, bvx)) < 0.5);
    input_tensor_values_[34] = ball_towards_me ? 1.0f : 0.0f;

    return true;
}

void AiParser::publish_action(unsigned int robot_id, const float* action_data)
{
    auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
    oxebots_interfaces::msg::RobotCmdData data;

    data.id = robot_id;
    data.x_velocity = action_data[0] * 1.5f;
    data.y_velocity = action_data[1] * 1.5f;
    data.angular_velocity = action_data[2] * 5.0f;
    data.kick_speed = action_data[3] > 0.0f ? 3.0f : 0.0f;

    cmd_msg->robots.push_back(data);
    cmd_pub_->publish(std::move(cmd_msg));
}

void AiParser::onHalted()
{
    unsigned int robot_id;
    if (getInput<unsigned int>("robot_id", robot_id)) {
        auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
        oxebots_interfaces::msg::RobotCmdData data;
        data.id = robot_id;
        data.x_velocity = 0.0f;
        data.y_velocity = 0.0f;
        data.angular_velocity = 0.0f;
        data.kick_speed = 0.0f;
        cmd_msg->robots.push_back(data);
        cmd_pub_->publish(std::move(cmd_msg));
    }
}

}