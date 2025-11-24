#include "oxebots_strategy/movement_calculation.h" 

#include <algorithm> 
#include <cmath>    

namespace {
double normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
} // namespace


PotentialFieldNode::PotentialFieldNode() : Node("movement_calculation_node") {
    
    this->declare_parameter("robot_id", 0);
    this->get_parameter("robot_id", robot_id_);
    this->declare_parameter("max_linear_speed", 1.0);

    this->declare_parameter("is_yellow", false);
    this->get_parameter("is_yellow", is_yellow_team_);
    
    // Calcular as posições dos gols com base na cor do time
    my_goal_x_ = is_yellow_team_ ? 2200.0 : -2200.0;
    opponent_goal_x_ = is_yellow_team_ ? -2200.0 : 2200.0;

    this->declare_parameter("p_gain_linear", 0.5); // Usado para escalar a velocidade final

    this->declare_parameter("max_angular_speed", 4.0);
    this->declare_parameter("p_gain_angular", 1.5);
    this->declare_parameter("angle_tolerance", 0.1);

    this->declare_parameter("attractive_gain", 1.0); 
    this->declare_parameter("repulsive_gain", 2.0);   
    this->declare_parameter("repulsive_radius", 500.0); 

    auto qosData = 10;
    auto qosPose = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    cmd_vel_pub_ = this->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", 10);
    game_data_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
        "/game_data", qosData, std::bind(&PotentialFieldNode::game_data_callback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotGoal>(
        "/robot_goal", qosPose, std::bind(&PotentialFieldNode::goal_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PotentialFieldNode::calculate_and_move, this));
    RCLCPP_INFO(this->get_logger(), "Nó de Campo Potencial (com Controle Angular) iniciado para o robô %d. Time %s, meu gol em X: %.1f", robot_id_, is_yellow_team_ ? "amarelo" : "azul", my_goal_x_);
}


void PotentialFieldNode::goal_callback(const oxebots_interfaces::msg::RobotGoal::SharedPtr msg) {
    
    if (msg->robot_id != static_cast<int>(robot_id_)) {
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_pos_ = movement::Coordinate{
        (float)msg->pose.pose.position.x, 
        (float)msg->pose.pose.position.y  
    };
    const auto& q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    target_w_ = std::atan2(siny_cosp, cosy_cosp);
}


void PotentialFieldNode::game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_game_data_ = msg; 
    game_data_received_ = (last_game_data_ != nullptr);
}


void PotentialFieldNode::calculate_and_move()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!target_pos_.has_value() || !target_w_.has_value() || !game_data_received_ || !last_game_data_) {
        return; 
    }

    std::optional<movement::Coordinate> current_pos_opt;
    std::vector<movement::Coordinate> obstacles;
    for (const auto& ally : last_game_data_->robots.allies) {
        if (ally.id == robot_id_) {
            current_pos_opt = movement::Coordinate{ally.x, ally.y, ally.orientation};
        } else {
            obstacles.push_back(movement::Coordinate{ally.x, ally.y, ally.orientation});
        }
    }
    for (const auto& enemy : last_game_data_->robots.enemies) {
        obstacles.push_back(movement::Coordinate{enemy.x, enemy.y, enemy.orientation});
    }

    if (!current_pos_opt) {
        return; 
    }
    movement::Coordinate current_pos = *current_pos_opt; 

    
    movement::Coordinate ball_pos{
        last_game_data_->ball.x, 
        last_game_data_->ball.y, 
        0.0
    };

    double vec_RT_x = target_pos_->x - current_pos.x;
    double vec_RT_y = target_pos_->y - current_pos.y;
    double vec_RB_x = ball_pos.x - current_pos.x;
    double vec_RB_y = ball_pos.y - current_pos.y;
    double dot_product = (vec_RT_x * vec_RB_x) + (vec_RT_y * vec_RB_y);
    double dist_RT_sq = (vec_RT_x * vec_RT_x) + (vec_RT_y * vec_RT_y);
    double dist_RB_sq = (vec_RB_x * vec_RB_x) + (vec_RB_y * vec_RB_y);

    if (dot_product > 0 && dist_RB_sq < dist_RT_sq && dist_RT_sq > (150.0 * 150.0)) {
        obstacles.push_back(ball_pos);
    }
    
    // --- Lógica de restrição de área do gol ---
    
    // Função auxiliar para ajustar o alvo se ele estiver dentro de uma área restrita
    auto adjust_target_if_in_area = [&](double area_x_min, double area_x_max, double area_y_min, double area_y_max, const std::string& area_name) {
        bool is_target_in_area =
            target_pos_->x >= area_x_min && target_pos_->x <= area_x_max &&
            target_pos_->y >= area_y_min && target_pos_->y <= area_y_max;

        if (is_target_in_area) {
            // Ajustar o target_pos_ para a borda mais próxima da área do gol
            double dist_to_min_x = std::abs(target_pos_->x - area_x_min);
            double dist_to_max_x = std::abs(target_pos_->x - area_x_max);
            double dist_to_min_y = std::abs(target_pos_->y - area_y_min);
            double dist_to_max_y = std::abs(target_pos_->y - area_y_max);

            double min_dist = std::min({dist_to_min_x, dist_to_max_x, dist_to_min_y, dist_to_max_y});

            if (min_dist == dist_to_min_x) {
                target_pos_->x = area_x_min - 100.0; // Empurra para fora
            } else if (min_dist == dist_to_max_x) {
                target_pos_->x = area_x_max + 100.0; // Empurra para fora
            } else if (min_dist == dist_to_min_y) {
                target_pos_->y = area_y_min - 100.0; // Empurra para fora
            } else {
                target_pos_->y = area_y_max + 100.0; // Empurra para fora
            }
            RCLCPP_WARN(this->get_logger(), "Robô %d: Alvo ajustado para fora da %s.", robot_id_, area_name.c_str());
        }
    };

    // Definir áreas
    double goal_area_y_min = -675.0; // 1350mm / 2
    double goal_area_y_max = 675.0;

    // Área Aliada
    double my_goal_area_x_min = is_yellow_team_ ? my_goal_x_ - 500.0 : my_goal_x_;
    double my_goal_area_x_max = is_yellow_team_ ? my_goal_x_ : my_goal_x_ + 500.0;

    // Área Inimiga
    double opponent_goal_area_x_min = is_yellow_team_ ? opponent_goal_x_ : opponent_goal_x_ - 500.0;
    double opponent_goal_area_x_max = is_yellow_team_ ? opponent_goal_x_ + 500.0 : opponent_goal_x_;

    // Restrição 1: Nenhum robô pode entrar na área inimiga
    adjust_target_if_in_area(opponent_goal_area_x_min, opponent_goal_area_x_max, goal_area_y_min, goal_area_y_max, "área inimiga");

    // Restrição 2: Apenas o goleiro (ID 0) pode entrar na área aliada
    if (robot_id_ != 0) {
        adjust_target_if_in_area(my_goal_area_x_min, my_goal_area_x_max, goal_area_y_min, goal_area_y_max, "área aliada");
    }

    auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
    auto cmd_data = oxebots_interfaces::msg::RobotCmdData();
    cmd_data.id = robot_id_;

    double p_gain_linear = this->get_parameter("p_gain_linear").as_double();
    double max_linear = this->get_parameter("max_linear_speed").as_double();
    double p_gain_angular = this->get_parameter("p_gain_angular").as_double();
    double max_angular = this->get_parameter("max_angular_speed").as_double();
    double angle_tol = this->get_parameter("angle_tolerance").as_double();

    double k_att = this->get_parameter("attractive_gain").as_double();
    double k_rep = this->get_parameter("repulsive_gain").as_double();
    double d0_rep = this->get_parameter("repulsive_radius").as_double();

    // === 1. CÁLCULO DE VELOCIDADE LINEAR (vx, vy) ===
    double distance_to_target = movement::calculateDistance(current_pos, *target_pos_);

    if (distance_to_target < 100.0) { 
         cmd_data.x_velocity = 0.0;
         cmd_data.y_velocity = 0.0;
    } else {
        double desired_speed = std::min(max_linear, distance_to_target * p_gain_linear);
        movement::PotentialField pf_calculator;

        std::vector<double> force = pf_calculator.calculate(
            current_pos, 
            *target_pos_, 
            obstacles,
            k_att,
            k_rep,
            d0_rep
        );
        
        double force_magnitude = std::hypot(force[0], force[1]);
        
        if (force_magnitude > 0.01) { 
            cmd_data.x_velocity = (force[0] / force_magnitude) * desired_speed;
            cmd_data.y_velocity = (force[1] / force_magnitude) * desired_speed;
        } else {
             cmd_data.x_velocity = 0.0;
             cmd_data.y_velocity = 0.0;
        }
    }

    // === 2. CÁLCULO DE VELOCIDADE ANGULAR (vw) ===
    double current_w = current_pos.orientation; 
    double target_w = *target_w_;                
    double angle_error = normalizeAngle(target_w - current_w);

    if (std::abs(angle_error) < angle_tol) {
        cmd_data.angular_velocity = 0.0; 
    } else {
        double desired_vw = angle_error * p_gain_angular;
        cmd_data.angular_velocity = std::clamp(desired_vw, -max_angular, max_angular);
    }

    // === 3. PUBLICA O COMANDO COMPLETO ===
    cmd_msg->robots.push_back(cmd_data);
    cmd_vel_pub_->publish(std::move(cmd_msg));
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PotentialFieldNode>());
    rclcpp::shutdown();
    return 0;
}