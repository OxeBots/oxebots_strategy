#include "oxebots_strategy/movement_calculation.h"

PotentialFieldNode::PotentialFieldNode() : Node("movement_calculation_node") {
    this->declare_parameter("robot_id", 0);
    this->get_parameter("robot_id", robot_id_);

    this->declare_parameter("max_linear_speed", 1.0); // m/s
    this->declare_parameter("p_gain_linear", 0.5);
    this->declare_parameter("max_angular_speed", 4.0); // rad/s

    auto qosData = 10;
    auto qosPose = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    cmd_vel_pub_ = this->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", 10);
    game_data_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
        "/game_data", qosData, std::bind(&PotentialFieldNode::game_data_callback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", qosPose, std::bind(&PotentialFieldNode::goal_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PotentialFieldNode::calculate_and_move, this));
    
    RCLCPP_INFO(this->get_logger(), "Nó de Campo Potencial iniciado para o robô %d.", robot_id_);
}

void PotentialFieldNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_pos_ = movement::Coordinate{
        (float)msg->pose.position.x, 
        (float)msg->pose.position.y
    };
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Novo alvo recebido: (%.2f, %.2f)", target_pos_->x, target_pos_->y);
}

void PotentialFieldNode::game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_game_data_ = msg; // Apenas armazena a última mensagem recebida
    if(last_game_data_){
        game_data_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Recebi GameData: %d",game_data_received_);
    }
    else{
        game_data_received_ = false;
    RCLCPP_INFO(this->get_logger(), "Não Recebi GameData: %d",game_data_received_);

    }
}

void PotentialFieldNode::calculate_and_move()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!target_pos_.has_value() || !game_data_received_ || !last_game_data_) {
        return; // Precisa de alvo e dados do jogo
    }

    // Encontra o robô e os obstáculos na última mensagem recebida
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

    // Se não encontrou nosso robô, não faz nada
    if (!current_pos_opt) {
        return;
    }
    movement::Coordinate current_pos = *current_pos_opt;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[DEBUG] Using current_pos: (%.2f, %.2f) to target: (%.2f, %.2f)", current_pos.x, current_pos.y, target_pos_->x, target_pos_->y);

    auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
    auto cmd_data = oxebots_interfaces::msg::RobotCmdData();
    cmd_data.id = robot_id_;

    // P-Controller para velocidade linear
    double distance_to_target = movement::calculateDistance(current_pos, *target_pos_);
    double p_gain = this->get_parameter("p_gain_linear").as_double();
    double max_linear = this->get_parameter("max_linear_speed").as_double();
    double desired_speed = std::min(max_linear, distance_to_target * p_gain);

    // Se estiver muito perto, considera que chegou e para.
    if (distance_to_target < 100.0) { // Limiar de 10cm para parada total
         cmd_data.x_velocity = 0.0;
         cmd_data.y_velocity = 0.0;
         cmd_data.angular_velocity = 0.0;
         target_pos_.reset(); // Para de se mover até receber novo alvo
         RCLCPP_INFO(this->get_logger(), "Alvo alcançado!");
    } else {
        movement::PotentialField pf_calculator;
        std::vector<double> force = pf_calculator.calculate(current_pos, *target_pos_, obstacles);
        
        double force_magnitude = std::hypot(force[0], force[1]);
        
        // Converte a força em velocidades no referencial do MUNDO, usando a velocidade proporcional
        double desired_vx = 0.0;
        double desired_vy = 0.0;
        if (force_magnitude > 0.01) { // Evita divisão por zero
            desired_vx = (force[0] / force_magnitude) * desired_speed;
            desired_vy = (force[1] / force_magnitude) * desired_speed;
        }

        cmd_data.x_velocity = desired_vx;
        cmd_data.y_velocity = desired_vy;

        // Robô omnidirecional não precisa girar para se mover
        cmd_data.angular_velocity = 0.0;
    }

    cmd_msg->robots.push_back(cmd_data);
    cmd_vel_pub_->publish(std::move(cmd_msg));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PotentialFieldNode>());
    rclcpp::shutdown();
    return 0;
}
