#include "oxebots_strategy/go_to_point_node.h"
#include <cmath>

namespace {
// Função auxiliar para normalizar o ângulo entre -PI e PI
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
}

namespace oxebots_strategy {

GoToPointNode::GoToPointNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node) {
    // Configuração do QoS (Quality of Service) para ser "Transient Local"
    // Isso garante que o nó que receber a mensagem pegue o último valor publicado, mesmo se tiver começado depois
    auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    
    // Publisher para enviar o comando de objetivo (pose) para o robô
    goal_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);
    
    // Subscriber para receber os dados do jogo (posições de robôs e bola)
    game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
        "/game_data", 10, std::bind(&GoToPointNode::gameDataCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "GoToPointNode pronto. Convertendo MM para Metros.");
}

// Portas do Behavior Tree (entradas e saídas de dados para o nó)
BT::PortsList GoToPointNode::providedPorts() {
    return { BT::InputPort<unsigned int>("robot_id"),
             BT::InputPort<double>("x"),
             BT::InputPort<double>("y"),
             BT::InputPort<double>("tolerance", -1.0, "Tolerância para sucesso (se <= 0, nunca retorna SUCCESS)") };
}

void GoToPointNode::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg) {
    last_game_data_ = msg;
}

void GoToPointNode::geometryDataCallback(const oxebots_interfaces::msg::SSLGeometryData::SharedPtr msg) {
    field_size_ = msg->field;
}

// Função auxiliar para buscar os dados específicos de um robô aliado pelo ID
std::optional<oxebots_interfaces::msg::RobotGameData> GoToPointNode::getRobotData(unsigned int robot_id) {
    if (!last_game_data_) return std::nullopt;
    for (const auto& ally : last_game_data_->robots.allies) {
        if (ally.id == robot_id) return ally;
    }
    return std::nullopt;
}

// Publica a mensagem de RobotGoal com a posição e orientação desejadas
void GoToPointNode::publishGoal() {
    double op_x, op_y;
    auto blackboard = config().blackboard;
    
    // Se a posição do gol adversário estiver no blackboard, faz o robô olhar para lá
    if (blackboard->get("opponent_goal_x", op_x) && blackboard->get("opponent_goal_y", op_y)) {
        target_w_ = std::atan2(op_y - target_pos_.y, op_x - target_pos_.x);
    } else {
        target_w_ = 0.0;
    }

    auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
    goal_msg->robot_id = robot_id_;
    goal_msg->pose.header.stamp = node_->now();
    goal_msg->pose.header.frame_id = "map"; 
    
    // Converte de milímetros (padrão interno) para metros (padrão do ROS/RobotGoal)
    goal_msg->pose.pose.position.x = target_pos_.x / 1000.0;
    goal_msg->pose.pose.position.y = target_pos_.y / 1000.0;
    
    // Converte o ângulo (target_w_) para um Quatérnio (representação de rotação em 3D)
    goal_msg->pose.pose.orientation.z = std::sin(target_w_ * 0.5);
    goal_msg->pose.pose.orientation.w = std::cos(target_w_ * 0.5);

    goal_pub_->publish(std::move(goal_msg));
}

// Chamado uma vez quando o nó BT é ativado
BT::NodeStatus GoToPointNode::onStart() {
    // Lê as entradas obrigatórias: ID do robô e as coordenadas do alvo
    if (!getInput<unsigned int>("robot_id", robot_id_)) return BT::NodeStatus::FAILURE;

    double tx, ty;
    if (!getInput<double>("x", tx) || !getInput<double>("y", ty)) return BT::NodeStatus::FAILURE;

    target_pos_.x = tx; 
    target_pos_.y = ty;

    bool is_yellow = node_->get_parameter("is_yellow_team").as_bool();

    // Limite de segurança para o robô não entrar dentro da área do gol
    if(is_yellow){

        if (target_pos_.x > 1740.0 && (target_pos_.y > -686 && target_pos_.y < 677) && robot_id_ == 2) {
        target_pos_.x = 1740.0;
        }
    }
     else {
       // Lógica para time Azul (inverte o sinal do X)
        if (target_pos_.x < -1740.0 && (target_pos_.y > -686 && target_pos_.y < 677) && robot_id_ == 2) {
            target_pos_.x = -1740.0;
        }
    }


    // double penalty_x = (field_size_->field_length / 2.0) - field_size_->penalty_area_depth;
    // double penalty_y_min = -field_size_->penalty_area_width / 2.0;
    // double penalty_y_max = field_size_->penalty_area_width / 2.0;

    // if(target_pos_.x > penalty_x && (target_pos_.y > penalty_y_min && target_pos_.y<penalty_y_max) && robot_id_ == 2){
    //     target_pos_.x = penalty_x;
    // }

    publishGoal();
    return BT::NodeStatus::RUNNING;
}

// Chamado em cada tick do Behavior Tree enquanto o nó estiver em RUNNING
BT::NodeStatus GoToPointNode::onRunning() {
    // Verifica se as coordenadas do alvo mudaram no Blackboard durante a execução
    double tx, ty;
    if (getInput<double>("x", tx) && getInput<double>("y", ty)) {
        double capped_x = tx;
        // Aplica a mesma restrição do onStart()
            
    bool is_yellow = node_->get_parameter("is_yellow_team").as_bool();

        // Limite de segurança para o robô não entrar dentro da área do gol
        if(is_yellow){

            if (tx > 1740.0 && (ty > -686 && ty < 677) && robot_id_ == 2) {
            capped_x = 1740.0;
            }
        }
        else {
        // Lógica para time Azul (inverte o sinal do X)
            if (tx < -1740.0 && (ty > -686 && ty < 677) && robot_id_ == 2) {
                capped_x = -1740.0;
            }
        }
        

        // Se houver uma mudança significativa (> 2mm), publica um novo objetivo
        if (std::abs(capped_x - target_pos_.x) > 2.0 || std::abs(ty - target_pos_.y) > 2.0) {
            target_pos_.x = capped_x;
            target_pos_.y = ty;
            publishGoal();
        }
    }

    // Pega a posição atual do robô para verificar se ele já chegou no destino
    auto robot = getRobotData(robot_id_);
    if (!robot) return BT::NodeStatus::RUNNING;

    // Calcula a distância euclidiana até o alvo
    double dist = std::hypot(robot->x - target_pos_.x, robot->y - target_pos_.y);
    
    double tolerance = -1.0;
    getInput<double>("tolerance", tolerance);

    // Se uma tolerância válida foi passada, verifica se o robô chegou perto o suficiente
    if (tolerance > 0.0) {
        bool pos_ok = (dist < tolerance);
        // Verifica se a orientação também está próxima do desejado (dentro de ~8.5 graus)
        bool ori_ok = (std::abs(normalizeAngle(target_w_ - robot->orientation)) < 0.15);
        if (pos_ok && ori_ok) {
            RCLCPP_INFO(node_->get_logger(), "Robô %d chegou ao alvo (dist: %.1f, tol: %.1f).", robot_id_, dist, tolerance);
            return BT::NodeStatus::SUCCESS;
        }
    }

    // Continua executando até chegar no destino ou ser interrompido
    return BT::NodeStatus::RUNNING;
}

void GoToPointNode::onHalted() {}

} // namespace oxebots_strategy