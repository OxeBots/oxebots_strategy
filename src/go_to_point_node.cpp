#include "oxebots_strategy/go_to_point_node.h"
#include <cmath>
#include <string>

namespace {
    // Constantes de controle
    const double K_P = 2.0; // Ganho proporcional para velocidade linear
    const double K_W = 3.0; // Ganho proporcional para velocidade angular
    const double LOOKAHEAD_DISTANCE = 0.2; // metros
    const double ROBOT_RADIUS = 0.09; // metros (para obstáculos)
    const double STOP_THRESHOLD = 0.1; // metros (10cm de tolerância)
    const double ORIENTATION_THRESHOLD = 0.15; // radianos

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
}

namespace oxebots_strategy {

GoToPointNode::GoToPointNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node) {
    cmd_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotCmd>("robot_commands", 10);
    
    game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
        "/game_data", 10, std::bind(&GoToPointNode::gameDataCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "GoToPointNode (RRT*) pronto.");
}

BT::PortsList GoToPointNode::providedPorts() {
    return { BT::InputPort<unsigned int>("robot_id"),
             BT::InputPort<double>("x"),
             BT::InputPort<double>("y") };
}

void GoToPointNode::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg) {
    last_game_data_ = msg;
}

std::optional<oxebots_interfaces::msg::RobotGameData> GoToPointNode::getRobotData(unsigned int robot_id) {
    if (!last_game_data_) return std::nullopt;
    for (const auto& ally : last_game_data_->robots.allies) {
        if (ally.id == robot_id) return ally;
    }
    return std::nullopt;
}

BT::NodeStatus GoToPointNode::onStart() {
    if (!getInput<unsigned int>("robot_id", robot_id_)) return BT::NodeStatus::FAILURE;

    double tx, ty;
    if (!getInput<double>("x", tx) || !getInput<double>("y", ty)) return BT::NodeStatus::FAILURE;
    
    // Armazena o alvo em metros
    target_pos_ = {tx / 1000.0, ty / 1000.0};

    double op_x, op_y;
    auto blackboard = config().blackboard;
    if (blackboard->get("opponent_goal_x", op_x) && blackboard->get("opponent_goal_y", op_y)) {
        target_w_ = std::atan2(op_y / 1000.0 - target_pos_.y, op_x / 1000.0 - target_pos_.x);
    } else {
        target_w_ = 0.0;
    }
    
    current_path_.clear();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPointNode::onRunning() {
    if (!last_game_data_) {
        RCLCPP_WARN(node_->get_logger(), "Aguardando dados do jogo...");
        return BT::NodeStatus::RUNNING;
    }

    auto robot_opt = getRobotData(robot_id_);
    if (!robot_opt) {
        RCLCPP_WARN(node_->get_logger(), "Aguardando dados do robô %d...", robot_id_);
        return BT::NodeStatus::RUNNING;
    }
    
    auto robot_data = *robot_opt;
    Point start_pos = {robot_data.x / 1000.0, robot_data.y / 1000.0};
    double robot_w = robot_data.orientation;

    // Verificar condição de sucesso
    double dist_to_target = std::hypot(start_pos.x - target_pos_.x, start_pos.y - target_pos_.y);
    double orient_error = std::abs(normalizeAngle(target_w_ - robot_w));

    if (dist_to_target < STOP_THRESHOLD && orient_error < ORIENTATION_THRESHOLD) {
        RCLCPP_INFO(node_->get_logger(), "Robô %d chegou ao alvo.", robot_id_);
        // Publicar comando de parada
        auto stop_cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
        oxebots_interfaces::msg::RobotCmdData robot_cmd;
        robot_cmd.id = robot_id_;
        robot_cmd.x_velocity = 0.0f;
        robot_cmd.y_velocity = 0.0f;
        robot_cmd.angular_velocity = 0.0f;
        robot_cmd.kick_speed = 0.0f;
        stop_cmd_msg->robots.push_back(robot_cmd);
        cmd_pub_->publish(std::move(stop_cmd_msg));
        return BT::NodeStatus::SUCCESS;
    }

    // --- Início do Planejamento RRT* e Controle ---

    // 1. Coletar obstáculos
    std::vector<Obstacle> obstacles;
    for (const auto& ally : last_game_data_->robots.allies) {
        if (ally.id != robot_id_) {
            obstacles.push_back({{ally.x / 1000.0, ally.y / 1000.0}, ROBOT_RADIUS});
        }
    }
    for (const auto& enemy : last_game_data_->robots.enemies) {
        obstacles.push_back({{enemy.x / 1000.0, enemy.y / 1000.0}, ROBOT_RADIUS});
    }

    // 2. Planejar a rota
    RRTStarPlanner planner(start_pos, target_pos_, obstacles, 0.15, 0.3, 0.5, 5000);
    current_path_ = planner.plan_path();
    std::string path_str = "";
    for (size_t i = 0; i < current_path_.size(); ++i) {
        path_str += "(" + std::to_string(current_path_[i].x) + ", " + std::to_string(current_path_[i].y) + ")";
        if (i < current_path_.size() - 1) {
            path_str += ", ";
        }
    }
    RCLCPP_INFO(node_->get_logger(), "RRT* Path planning: Start(%.2f, %.2f) -> Goal(%.2f, %.2f). Path found with %zu points. Path: [%s]", start_pos.x, start_pos.y, target_pos_.x, target_pos_.y, current_path_.size(), path_str.c_str());

    auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
    oxebots_interfaces::msg::RobotCmdData robot_cmd;
    robot_cmd.id = robot_id_;

    if (current_path_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "RRT*: Nenhum caminho encontrado. Parando robô %d.", robot_id_);
        robot_cmd.x_velocity = 0.0f;
        robot_cmd.y_velocity = 0.0f;
        robot_cmd.angular_velocity = 0.0f;
    } else {
        // 3. Lógica do Pure Pursuit
        lookahead_point_ = find_lookahead_point(current_path_, start_pos, LOOKAHEAD_DISTANCE);
        
        // 4. Calcular velocidades
        double dx_global = lookahead_point_.x - start_pos.x;
        double dy_global = lookahead_point_.y - start_pos.y;

        double target_vel_x_global = K_P * dx_global;
        double target_vel_y_global = K_P * dy_global;

        // Envia velocidades globais, como esperado pelo grSim_controller
        robot_cmd.x_velocity = target_vel_x_global;
        robot_cmd.y_velocity = target_vel_y_global;
        
        RCLCPP_INFO(node_->get_logger(), "Robot %d velocities: vx=%f, vy=%f", robot_id_, robot_cmd.x_velocity, robot_cmd.y_velocity);

        // Velocidade angular para apontar para o alvo final
        robot_cmd.angular_velocity = K_W * normalizeAngle(target_w_ - robot_w);
    }
    
    robot_cmd.kick_speed = 0.0f;
    cmd_msg->robots.push_back(robot_cmd);
    cmd_pub_->publish(std::move(cmd_msg));
    return BT::NodeStatus::RUNNING;
}

Point GoToPointNode::find_lookahead_point(const std::vector<Point>& path, const Point& robot_pos, double lookahead_distance) {
    if (path.empty()) return robot_pos;

    Point lookahead_pt = path.back(); // Por padrão, o alvo final
    if (path.size() > 1) {
        for (size_t i = path.size() - 1; i > 0; --i) {
            Point p1 = path[i-1];
            Point p2 = path[i];
            
            Point d = {p2.x - p1.x, p2.y - p1.y};
            Point f = {p1.x - robot_pos.x, p1.y - robot_pos.y};

            double a = d.x*d.x + d.y*d.y;
            double b = 2 * (f.x*d.x + f.y*d.y);
            double c = f.x*f.x + f.y*f.y - lookahead_distance*lookahead_distance;
            double discriminant = b*b - 4*a*c;

            if (discriminant >= 0) {
                discriminant = sqrt(discriminant);
                double t1 = (-b - discriminant) / (2*a);
                double t2 = (-b + discriminant) / (2*a);
                
                if (t1 >= 0 && t1 <= 1) {
                    return {p1.x + t1*d.x, p1.y + t1*d.y};
                }
                if (t2 >= 0 && t2 <= 1) {
                    return {p1.x + t2*d.x, p1.y + t2*d.y};
                }
            }
        }
    }
    return lookahead_pt;
}

void GoToPointNode::onHalted() {
    RCLCPP_INFO(node_->get_logger(), "GoToPointNode (RRT*) Halted. Parando robô %d", robot_id_);
    auto stop_cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
    oxebots_interfaces::msg::RobotCmdData robot_cmd;
    robot_cmd.id = robot_id_;
    robot_cmd.x_velocity = 0.0f;
    robot_cmd.y_velocity = 0.0f;
    robot_cmd.angular_velocity = 0.0f;
    robot_cmd.kick_speed = 0.0f;
    stop_cmd_msg->robots.push_back(robot_cmd);
    cmd_pub_->publish(std::move(stop_cmd_msg));
}

} // namespace oxebots_strategy