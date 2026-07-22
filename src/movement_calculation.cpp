#include "oxebots_strategy/movement_calculation.h" 
#include <algorithm> 
#include <cmath>    

namespace {
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
}

PathFollowerNode::PathFollowerNode() : Node("path_follower_node") {
    this->declare_parameter("robot_id", 0);
    this->get_parameter("robot_id", robot_id_);
    
    this->declare_parameter("max_linear_speed", 1.0);
    this->declare_parameter("p_gain_linear", 2.0);
    this->declare_parameter("max_angular_speed", 4.0);
    this->declare_parameter("p_gain_angular", 3.0);
    this->declare_parameter("angle_tolerance", 0.1);
    this->declare_parameter("lookahead_distance", 300.0); // mm

    std::string robot_prefix = "/robot_" + std::to_string(robot_id_);
    
    cmd_vel_pub_ = this->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", 10);
    
    game_data_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
        "/game_data", 10, std::bind(&PathFollowerNode::game_data_callback, this, std::placeholders::_1));
    
    goal_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotGoal>(
        "/robot_goal", 10, std::bind(&PathFollowerNode::goal_callback, this, std::placeholders::_1));
    
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        robot_prefix + "/path", 10, std::bind(&PathFollowerNode::path_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&PathFollowerNode::calculate_and_move, this));
    
    RCLCPP_INFO(this->get_logger(), "Seguidor de Caminho iniciado para o robô %d ouvindo %s/path", robot_id_, robot_prefix.c_str());
}

void PathFollowerNode::goal_callback(const oxebots_interfaces::msg::RobotGoal::SharedPtr msg) {
    if (msg->robot_id != robot_id_) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_goal_ = movement::Coordinate{
        (float)(msg->pose.pose.position.x * 1000.0), 
        (float)(msg->pose.pose.position.y * 1000.0)  
    };
    const auto& q = msg->pose.pose.orientation;
    target_w_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void PathFollowerNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_path_ = msg;
}

void PathFollowerNode::game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_game_data_ = msg; 
    game_data_received_ = true;
}

void PathFollowerNode::calculate_and_move() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!game_data_received_ || !last_game_data_) {
        return; 
    }

    movement::Coordinate current_pos;
    bool found_me = false;
    for (const auto& ally : last_game_data_->robots.allies) {
        if (static_cast<int>(ally.id) == robot_id_) {
            current_pos = {ally.x, ally.y, ally.orientation};
            found_me = true;
            break;
        }
    }
    if (!found_me) return;

    movement::Coordinate target_pt;
    bool target_found = false;

    // --- Seguir Rota do D* ---
    if (last_path_ && !last_path_->poses.empty()) {
            double lookahead_dist = this->get_parameter("lookahead_distance").as_double();
            for (const auto& pose_stamped : last_path_->poses) {
                float px = pose_stamped.pose.position.x * 1000.0f;
                float py = pose_stamped.pose.position.y * 1000.0f;
                double d = std::hypot(px - current_pos.x, py - current_pos.y);
                
                if (d > lookahead_dist) {
                    target_pt = {px, py, 0.0f};
                    target_found = true;
                    break;
                }
            }
            if (!target_found) {
                target_pt = {
                    (float)last_path_->poses.back().pose.position.x * 1000.0f,
                    (float)last_path_->poses.back().pose.position.y * 1000.0f,
                    0.0f
                };
                target_found = true;
            }
        } else if (last_path_ && last_path_->poses.empty()) {
            // Rota vazia recebida: parar o robô
            auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
            oxebots_interfaces::msg::RobotCmdData cmd_data;
            cmd_data.id = robot_id_;
            cmd_data.x_velocity = 0.0;
            cmd_data.y_velocity = 0.0;
            cmd_data.angular_velocity = 0.0;
            cmd_msg->robots.push_back(cmd_data);
            cmd_vel_pub_->publish(std::move(cmd_msg));
            return;
        }

    if (!target_found) return;

    auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
    oxebots_interfaces::msg::RobotCmdData cmd_data;
    cmd_data.id = robot_id_;

    // --- Controle Angular (calculado antes do linear: a rotação tem prioridade) ---
    double angle_err = 0.0;
    bool has_target_w = target_w_.has_value();
    if (has_target_w) {
        angle_err = normalizeAngle(*target_w_ - current_pos.orientation);
        if (std::abs(angle_err) < this->get_parameter("angle_tolerance").as_double()) {
            cmd_data.angular_velocity = 0.0;
        } else {
            double p_ang = this->get_parameter("p_gain_angular").as_double();
            double max_ang = this->get_parameter("max_angular_speed").as_double();
            cmd_data.angular_velocity = std::clamp(angle_err * p_ang, -max_ang, max_ang);
        }
    }

    // --- Controle Linear ---
    double dist_to_target = std::hypot(target_pt.x - current_pos.x, target_pt.y - current_pos.y);

    // Se tivermos um alvo final, checamos se chegamos nele
    double check_dist = target_goal_.has_value() ?
        std::hypot(target_goal_->x - current_pos.x, target_goal_->y - current_pos.y) : dist_to_target;

    if (check_dist < 40.0 || dist_to_target < 0.001) {
        cmd_data.x_velocity = 0.0;
        cmd_data.y_velocity = 0.0;
    } else {
        double p_lin = this->get_parameter("p_gain_linear").as_double();
        double max_lin = this->get_parameter("max_linear_speed").as_double();

        double vx = (target_pt.x - current_pos.x) / dist_to_target;
        double vy = (target_pt.y - current_pos.y) / dist_to_target;

        double speed = std::min(max_lin, (dist_to_target / 1000.0) * p_lin);

        // Prioriza alinhar antes de avançar: com erro angular >= 90 graus a velocidade linear
        // vai a zero (gira no lugar); alinhado (erro ~0) mantém a velocidade cheia. Evita que o
        // robô empurre/bata na bola fora de posição enquanto ainda está girando para encará-la.
        if (has_target_w) {
            double alignment_factor = std::max(0.0, std::cos(angle_err));
            speed *= alignment_factor;
        }

        cmd_data.x_velocity = vx * speed;
        cmd_data.y_velocity = vy * speed;
    }

    cmd_msg->robots.push_back(cmd_data);
    cmd_vel_pub_->publish(std::move(cmd_msg));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
