#include "oxebots_strategy/movement_calculation.h" 
#include <algorithm> 
#include <cmath>    

namespace {
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
double normalizeAngleDeg(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
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
    status_pub_ = this->create_publisher<oxebots_interfaces::msg::RobotMotionStatus>("/robot_motion_status", 10);

    game_data_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
        "/game_data", 10, std::bind(&PathFollowerNode::game_data_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotGoal>(
        "/robot_goal", 10, std::bind(&PathFollowerNode::goal_callback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        robot_prefix + "/path", 10, std::bind(&PathFollowerNode::path_callback, this, std::placeholders::_1));

    override_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotMotionOverride>(
        "/robot_motion_override", 10, std::bind(&PathFollowerNode::override_callback, this, std::placeholders::_1));

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

void PathFollowerNode::override_callback(const oxebots_interfaces::msg::RobotMotionOverride::SharedPtr msg) {
    if (static_cast<int>(msg->robot_id) != robot_id_) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_override_msg_ = msg;
    last_override_time_ = std::chrono::steady_clock::now();
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

    // --- Canal de override (Align/Kick/Halt) ---
    // Único dono de /robot_commands para este robô: AlignToBallNode, KickBallNode e a ação Halt
    // (strategy_node.cpp) não publicam mais lá diretamente, só pedem um modo aqui via
    // /robot_motion_override. Isso elimina a corrida de publishers que causava vários bugs
    // (ver histórico em git log) — três/quatro nós independentes escrevendo no mesmo tópico sem
    // coordenação. O modo cai para NONE (comportamento padrão de seguir caminho, abaixo) tanto
    // quando o leaf libera explicitamente quanto quando um override simplesmente para de chegar
    // por mais que kOverrideTtl (proteção para casos como Halt, que não tem callback de saída).
    uint8_t effective_mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_NONE;
    if (last_override_msg_ && (std::chrono::steady_clock::now() - last_override_time_) < kOverrideTtl) {
        effective_mode = last_override_msg_->mode;
    }

    if (effective_mode == oxebots_interfaces::msg::RobotMotionOverride::MODE_HALT) {
        publishHalt();
        return;
    }
    if (effective_mode == oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK) {
        runKick();
        return;
    }
    if (effective_mode == oxebots_interfaces::msg::RobotMotionOverride::MODE_ALIGN_TO_BALL ||
        effective_mode == oxebots_interfaces::msg::RobotMotionOverride::MODE_ALIGN_TO_ANGLE) {
        runAlign(current_pos, effective_mode);
        return;
    }

    // --- Comportamento padrão: seguir caminho (D*/linha reta) + apontar para o gol ---
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

    // --- Controle Linear (distância ao alvo, calculada antes do controle angular) ---
    double dist_to_target = std::hypot(target_pt.x - current_pos.x, target_pt.y - current_pos.y);

    // Se tivermos um alvo final, checamos se chegamos nele
    double check_dist = target_goal_.has_value() ?
        std::hypot(target_goal_->x - current_pos.x, target_goal_->y - current_pos.y) : dist_to_target;

    // 10mm: precisa ficar ABAIXO da menor tolerância usada em qualquer GoToPoint das árvores
    // (20mm na fase 2 de master_strategy.xml, a aproximação final antes do chute). Estava em
    // 40mm, maior que essa tolerância de 20mm — o robô era mandado parar (velocidade zerada)
    // antes de conseguir satisfazer a própria condição de sucesso do GoToPoint (dist < 20mm),
    // travando pra sempre alguns mm fora do alcance (confirmado no log: "dist=28.5" congelado
    // por 8+ segundos, GoToPoint nunca retornando SUCCESS, robô parado na frente da bola).
    constexpr double kArrivedDistanceMm = 10.0;
    bool arrived = check_dist < kArrivedDistanceMm || dist_to_target < 0.001;

    // --- Controle Angular ---
    // Uma vez que o robô "chegou" (arrived), este nó para de disputar autoridade angular: sem
    // isso, target_w_ (calculado ao publicar o goal, geralmente virando para o gol) continuava
    // sendo perseguido para sempre a cada tick, mesmo depois do GoToPoint já ter sucesso — e
    // brigava pelo mesmo /robot_commands com o AlignToBallNode, que gira o robô para a bola REAL
    // logo em seguida. Dois controladores de rotação com alvos diferentes disputando o mesmo
    // tópico faziam o robô nunca convergir (visto no log: preso 44s sem nunca alinhar). Zerar
    // target_w_ aqui cede a rotação para quem realmente precisa dela nesse momento.
    double angle_err = 0.0;
    bool has_target_w = target_w_.has_value();
    if (arrived) {
        cmd_data.angular_velocity = 0.0;
        target_w_.reset();
        has_target_w = false;
    } else if (has_target_w) {
        angle_err = normalizeAngle(*target_w_ - current_pos.orientation);
        if (std::abs(angle_err) < this->get_parameter("angle_tolerance").as_double()) {
            cmd_data.angular_velocity = 0.0;
        } else {
            double p_ang = this->get_parameter("p_gain_angular").as_double();
            double max_ang = this->get_parameter("max_angular_speed").as_double();
            cmd_data.angular_velocity = std::clamp(angle_err * p_ang, -max_ang, max_ang);
        }
    }

    if (arrived) {
        cmd_data.x_velocity = 0.0;
        cmd_data.y_velocity = 0.0;
    } else {
        double p_lin = this->get_parameter("p_gain_linear").as_double();
        double max_lin = this->get_parameter("max_linear_speed").as_double();

        double vx = (target_pt.x - current_pos.x) / dist_to_target;
        double vy = (target_pt.y - current_pos.y) / dist_to_target;

        double speed = std::min(max_lin, (dist_to_target / 1000.0) * p_lin);

        // Prioriza alinhar antes de avançar, mas só perto do alvo final: com erro angular >= 90
        // graus a velocidade linear vai a zero (gira no lugar); alinhado (erro ~0) mantém a
        // velocidade cheia. Evita que o robô empurre/bata na bola fora de posição enquanto ainda
        // está girando para encará-la. Longe do alvo (perseguindo a bola pelo campo) o robô ainda
        // não precisa estar de frente para o gol, então corre em velocidade plena sem ser freado
        // por essa checagem. Com o ataque em duas fases (master_strategy.xml), a "arrancada final"
        // do ponto de pré-chute até o ponto de captura tem só ~270mm — um range de 400mm cobriria
        // essa corrida inteira e deixaria o robô lento bem quando ele já devia estar alinhado e
        // só precisa avançar reto. 100mm reduz o freio para só os últimos centímetros.
        constexpr double kAlignmentPriorityRangeMm = 100.0;
        if (has_target_w && check_dist < kAlignmentPriorityRangeMm) {
            double alignment_factor = std::max(0.0, std::cos(angle_err));
            speed *= alignment_factor;
        }

        cmd_data.x_velocity = vx * speed;
        cmd_data.y_velocity = vy * speed;
    }

    cmd_msg->robots.push_back(cmd_data);
    cmd_vel_pub_->publish(std::move(cmd_msg));
}

void PathFollowerNode::publishStatus(uint8_t active_mode, bool aligned) {
    auto msg = std::make_unique<oxebots_interfaces::msg::RobotMotionStatus>();
    msg->robot_id = robot_id_;
    msg->active_mode = active_mode;
    msg->aligned = aligned;
    msg->kick_fire_count = kick_fire_count_;
    status_pub_->publish(std::move(msg));
}

void PathFollowerNode::publishHalt() {
    auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
    oxebots_interfaces::msg::RobotCmdData cmd_data;
    cmd_data.id = robot_id_;
    cmd_data.x_velocity = 0.0;
    cmd_data.y_velocity = 0.0;
    cmd_data.angular_velocity = 0.0;
    cmd_msg->robots.push_back(cmd_data);
    cmd_vel_pub_->publish(std::move(cmd_msg));
    publishStatus(oxebots_interfaces::msg::RobotMotionOverride::MODE_HALT, false);
}

// Gira o robô no lugar até apontar para o alvo pedido (a bola real, para MODE_ALIGN_TO_BALL, ou
// um ângulo fixo, para MODE_ALIGN_TO_ANGLE). Portado de AlignToBallNode: a diferença é que aqui
// roda no timer de 50Hz do controlador em vez de no tick do BT, e é o único a escrever em
// /robot_commands enquanto este modo estiver ativo.
void PathFollowerNode::runAlign(const movement::Coordinate& current_pos, uint8_t mode) {
    double target_angle_deg;
    if (mode == oxebots_interfaces::msg::RobotMotionOverride::MODE_ALIGN_TO_BALL) {
        double dx = last_game_data_->ball.x - current_pos.x;
        double dy = last_game_data_->ball.y - current_pos.y;
        target_angle_deg = std::atan2(dy, dx) * 180.0 / M_PI;
    } else {
        target_angle_deg = last_override_msg_->target_angle * 180.0 / M_PI;
    }

    double heading_deg = current_pos.orientation * 180.0 / M_PI;
    double offset_deg = normalizeAngleDeg(target_angle_deg - heading_deg);
    bool aligned = std::abs(offset_deg) < last_override_msg_->angle_threshold_deg;

    auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
    oxebots_interfaces::msg::RobotCmdData cmd_data;
    cmd_data.id = robot_id_;
    cmd_data.x_velocity = 0.0;
    cmd_data.y_velocity = 0.0;
    if (aligned) {
        cmd_data.angular_velocity = 0.0;
    } else {
        double offset_rad = offset_deg * M_PI / 180.0;
        cmd_data.angular_velocity = std::clamp(
            offset_rad * last_override_msg_->p_gain,
            -last_override_msg_->max_angular_speed, last_override_msg_->max_angular_speed);
    }
    cmd_msg->robots.push_back(cmd_data);
    cmd_vel_pub_->publish(std::move(cmd_msg));

    // Log throttled a 150ms (~10Hz): a 1000ms anterior estava mascarando o problema por
    // "aliasing" — um caso real de alinhamento que nunca convergia (erro ~82° girando sem parar
    // por 9s, confirmado com bola PARADA, não perseguindo alvo em movimento) aparecia no log
    // como um offset "constante" porque a amostragem de 1Hz calhava de cair quase sempre na
    // mesma fase da rotação. Com 150ms dá pra ver ~10 amostras dentro da janela do Timeout de
    // 1.5s e distinguir se é giro contínuo numa direção só (bug de sinal) ou oscilação por
    // ultrapassar o alvo (ganho alto + atraso de dados).
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 150,
        "Robô %d AlignTo(modo=%d): heading=%.1fdeg alvo=%.1fdeg offset=%.1fdeg limite=%.1fdeg alinhado=%s cmd_ang=%.2f",
        robot_id_, mode, heading_deg, target_angle_deg, offset_deg,
        last_override_msg_->angle_threshold_deg, aligned ? "sim" : "nao", cmd_data.angular_velocity);

    publishStatus(mode, aligned);
}

// Máquina de estados do chute, portada de KickBallNode. Enquanto MODE_KICK estiver ativo:
// dispara (avanço + kick_speed) por kKickFireWindow, incrementa kick_fire_count_ ao terminar a
// janela, e então segura kKickCooldown antes de aceitar outro pedido — sem isso, um pedido de
// chute que continue chegando (o leaf reenvia a cada tick enquanto RUNNING) dispararia de novo
// instantaneamente, virando uma "metralhadora" que nunca solta a bola direito.
void PathFollowerNode::runKick() {
    auto now = std::chrono::steady_clock::now();

    if (kick_phase_ == KickPhase::IDLE && now >= kick_cooldown_until_) {
        kick_phase_ = KickPhase::FIRING;
        kick_start_time_ = now;
    }

    if (kick_phase_ == KickPhase::FIRING) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - kick_start_time_);
        if (elapsed < kKickFireWindow) {
            auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
            oxebots_interfaces::msg::RobotCmdData cmd_data;
            cmd_data.id = robot_id_;
            cmd_data.kick_speed = last_override_msg_->kick_speed;
            cmd_data.x_velocity = 0.5; // avançar enquanto chuta, para garantir contato
            cmd_data.y_velocity = 0.0;
            cmd_data.angular_velocity = 0.0;
            cmd_msg->robots.push_back(cmd_data);
            cmd_vel_pub_->publish(std::move(cmd_msg));
            publishStatus(oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK, false);
            return;
        }

        kick_phase_ = KickPhase::IDLE;
        kick_cooldown_until_ = now + kKickCooldown;
        kick_fire_count_++;
    }

    // Fora da janela de disparo (aguardando pedido ou em cooldown): mantém o robô parado.
    auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
    oxebots_interfaces::msg::RobotCmdData cmd_data;
    cmd_data.id = robot_id_;
    cmd_data.x_velocity = 0.0;
    cmd_data.y_velocity = 0.0;
    cmd_data.angular_velocity = 0.0;
    cmd_msg->robots.push_back(cmd_data);
    cmd_vel_pub_->publish(std::move(cmd_msg));
    publishStatus(oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK, false);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
