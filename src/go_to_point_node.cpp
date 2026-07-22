#include "oxebots_strategy/go_to_point_node.h"
#include <cmath>

namespace {
// Função auxiliar para normalizar o ângulo entre -PI e PI
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void clampFromPenaltyArea(double& tx, double& ty, double goal_x, double depth, double width, double margin) {
    double half_width = width / 2.0;
    if (goal_x > 0) {
        double area_left = goal_x - depth;
        if (tx > (area_left - margin) && std::abs(ty) < (half_width + margin)) {
            tx = area_left - margin;
        }
    } else {
        double area_right = goal_x + depth;
        if (tx < (area_right + margin) && std::abs(ty) < (half_width + margin)) {
            tx = area_right + margin;
        }
    }
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
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/robot_intent_markers", 10);
    
    // Subscriber para receber os dados do jogo (posições de robôs e bola)
    game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
        "/game_data", 10, std::bind(&GoToPointNode::gameDataCallback, this, std::placeholders::_1));

    auto geom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    geometry_sub_ = node_->create_subscription<oxebots_interfaces::msg::SSLGeometryData>(
        "/field_geometry", geom_qos, std::bind(&GoToPointNode::geometryDataCallback, this, std::placeholders::_1));
    
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
    publishMarkers();
}

void GoToPointNode::publishMarkers() {
    auto robot = getRobotData(robot_id_);
    if (!robot) return;

    // --- Determinar Cor Dinâmica Baseada no Papel ---
    float r = 0.5, g = 0.5, b = 0.5; // Cor Padrão (Cinza)
    std::string role_name = "NONE";
    
    bool is_gk;
    uint32_t attacker_id, defender_id;
    auto blackboard = config().blackboard;

    if (blackboard->get("is_goalkeeper", is_gk) && is_gk && robot_id_ == 0) {
        r = 0.1; g = 0.1; b = 0.1; role_name = "GK"; // Preto
    } else if (blackboard->get("attacker_id", attacker_id) && robot_id_ == attacker_id) {
        r = 1.0; g = 0.5; b = 0.0; role_name = "ATT"; // Laranja
    } else if (blackboard->get("defender_id", defender_id) && robot_id_ == defender_id) {
        r = 0.0; g = 1.0; b = 0.0; role_name = "DEF"; // Verde
    }

    // --- Marker 1: Esfera no Destino ---
    auto goal_marker = visualization_msgs::msg::Marker();
    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = node_->now();
    goal_marker.ns = "robot_target_" + std::to_string(robot_id_);
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.position.x = target_pos_.x / 1000.0;
    goal_marker.pose.position.y = target_pos_.y / 1000.0;
    goal_marker.pose.position.z = 0.05;
    goal_marker.scale.x = 0.18; goal_marker.scale.y = 0.18; goal_marker.scale.z = 0.18;
    goal_marker.color.a = 0.8; // Mais opaco
    goal_marker.color.r = r; goal_marker.color.g = g; goal_marker.color.b = b;
    marker_pub_->publish(goal_marker);

    // --- Marker 2: Seta do Robô para o Destino ---
    auto arrow_marker = visualization_msgs::msg::Marker();
    arrow_marker.header.frame_id = "map";
    arrow_marker.header.stamp = node_->now();
    arrow_marker.ns = "robot_intent_" + std::to_string(robot_id_);
    arrow_marker.id = 1;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point start, end;
    start.x = robot->x / 1000.0; start.y = robot->y / 1000.0; start.z = 0.05;
    end.x = target_pos_.x / 1000.0; end.y = target_pos_.y / 1000.0; end.z = 0.05;
    arrow_marker.points.push_back(start);
    arrow_marker.points.push_back(end);
    arrow_marker.scale.x = 0.03; arrow_marker.scale.y = 0.07; arrow_marker.scale.z = 0.1;
    arrow_marker.color.a = 0.9; // Mais opaco
    arrow_marker.color.r = r; arrow_marker.color.g = g; arrow_marker.color.b = b;
    marker_pub_->publish(arrow_marker);

    // --- Marker 3: Texto (ID e Role) Flutuando sobre o Robô ---
    auto text_marker = visualization_msgs::msg::Marker();
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = node_->now();
    text_marker.ns = "robot_text_" + std::to_string(robot_id_);
    text_marker.id = 2;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = robot->x / 1000.0;
    text_marker.pose.position.y = robot->y / 1000.0;
    text_marker.pose.position.z = 0.5; // Mais alto
    text_marker.scale.z = 0.25; // Fonte maior
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0;
    text_marker.text = "R" + std::to_string(robot_id_) + " [" + role_name + "]";
    marker_pub_->publish(text_marker);
}

// Chamado uma vez quando o nó BT é ativado
BT::NodeStatus GoToPointNode::onStart() {
    unsigned int input_id;
    if (!getInput<unsigned int>("robot_id", input_id)) return BT::NodeStatus::FAILURE;

    // Obter o ID do robô desta instância (setado no blackboard pelo StrategyNode)
    uint32_t my_id;
    if (config().blackboard->get("robot_id", my_id)) {
        if (input_id != my_id) {
            // Se o comando não é para este robô, retornamos SUCCESS imediatamente 
            // para não travar a árvore, mas não fazemos nada.
            return BT::NodeStatus::SUCCESS;
        }
    }
    
    robot_id_ = input_id;

    double tx, ty;
    if (!getInput<double>("x", tx) || !getInput<double>("y", ty)) return BT::NodeStatus::FAILURE;

    target_pos_.x = tx; 
    target_pos_.y = ty;

    // Limite de segurança dinâmico para o robô não entrar dentro da área do gol
    double my_goal_x = 2200.0;
    double opponent_goal_x = -2200.0;
    bool is_gk = false;
    double p_depth = 500.0;
    double p_width = 1350.0;
    double margin = 100.0; // mm

    auto blackboard = config().blackboard;
    if (blackboard) {
        (void)blackboard->get("my_goal_x", my_goal_x);
        (void)blackboard->get("opponent_goal_x", opponent_goal_x);
        (void)blackboard->get("is_goalkeeper", is_gk);
    }

    if (field_size_.has_value()) {
        p_depth = field_size_->penalty_area_depth;
        p_width = field_size_->penalty_area_width;
        double half_len = field_size_->field_length / 2.0;
        my_goal_x = (my_goal_x > 0) ? half_len : -half_len;
        opponent_goal_x = (opponent_goal_x > 0) ? half_len : -half_len;
    }

    // 1. Bloquear área adversária para TODOS os robôs (incluindo o goleiro)
    clampFromPenaltyArea(target_pos_.x, target_pos_.y, opponent_goal_x, p_depth, p_width, margin);

    // 2. Bloquear a própria área apenas se NÃO for o goleiro
    if (!is_gk) {
        clampFromPenaltyArea(target_pos_.x, target_pos_.y, my_goal_x, p_depth, p_width, margin);
    }

    publishGoal();
    return BT::NodeStatus::RUNNING;
}

// Chamado em cada tick do Behavior Tree enquanto o nó estiver em RUNNING
BT::NodeStatus GoToPointNode::onRunning() {
    // Verifica se as coordenadas do alvo mudaram no Blackboard durante a execução
    double tx, ty;
    if (getInput<double>("x", tx) && getInput<double>("y", ty)) {
        double my_goal_x = 2200.0;
        double opponent_goal_x = -2200.0;
        bool is_gk = false;
        double p_depth = 500.0;
        double p_width = 1350.0;
        double margin = 100.0; // mm

        auto blackboard = config().blackboard;
        if (blackboard) {
            (void)blackboard->get("my_goal_x", my_goal_x);
            (void)blackboard->get("opponent_goal_x", opponent_goal_x);
            (void)blackboard->get("is_goalkeeper", is_gk);
        }

        if (field_size_.has_value()) {
            p_depth = field_size_->penalty_area_depth;
            p_width = field_size_->penalty_area_width;
            double half_len = field_size_->field_length / 2.0;
            my_goal_x = (my_goal_x > 0) ? half_len : -half_len;
            opponent_goal_x = (opponent_goal_x > 0) ? half_len : -half_len;
        }

        double clamped_x = tx;
        double clamped_y = ty;

        // 1. Bloquear área adversária para TODOS os robôs (incluindo o goleiro)
        clampFromPenaltyArea(clamped_x, clamped_y, opponent_goal_x, p_depth, p_width, margin);

        // 2. Bloquear a própria área apenas se NÃO for o goleiro
        if (!is_gk) {
            clampFromPenaltyArea(clamped_x, clamped_y, my_goal_x, p_depth, p_width, margin);
        }
        
        // Se houver uma mudança significativa (> 2mm), publica um novo objetivo
        if (std::abs(clamped_x - target_pos_.x) > 2.0 || std::abs(clamped_y - target_pos_.y) > 2.0) {
            target_pos_.x = clamped_x;
            target_pos_.y = clamped_y;
            publishGoal();
        }
    }

    // Pega a posição atual do robô para verificar se ele já chegou no destino
    auto robot = getRobotData(robot_id_);
    if (!robot) return BT::NodeStatus::RUNNING;

    // O objetivo em si só é (re)publicado quando muda (bloco acima), respeitando o QoS
    // transient_local do tópico /robot_goal. Os marcadores são só visualização e podem
    // ser atualizados a cada tick para refletir a posição atual do robô.
    publishMarkers();

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
