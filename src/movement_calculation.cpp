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
    // --- Distância de Lookahead (Pure Pursuit) ---
    // Padrão calibrado: 100.0 mm (anteriormente 300.0 mm).
    // Motivo do ajuste:
    // - 300.0 mm causava corte excessivo de curvas (~80-100 mm de desvio radial interno),
    //   podendo colidir com adversários ao contorná-los e reduzindo a fidelidade de rotas.
    // - 100.0 mm equivale a 2 células do grid do D* (resolução de 50 mm em match_config.yaml),
    //   filtrando o efeito escada da grade sem oscilar e garantindo contornos limpos.
    // - Para testes analíticos de bancada/artigos com malha densa, pode ser reduzido até 45 mm via launch.
    this->declare_parameter("lookahead_distance", 100.0); // mm

    std::string robot_prefix = "/robot_" + std::to_string(robot_id_);
    
    cmd_vel_pub_ = this->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", rclcpp::SensorDataQoS());
    status_pub_ = this->create_publisher<oxebots_interfaces::msg::RobotMotionStatus>("/robot_motion_status", 10);

    game_data_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
        "/game_data", 10, std::bind(&PathFollowerNode::game_data_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotGoal>(
        "/robot_goal", 10, std::bind(&PathFollowerNode::goal_callback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        robot_prefix + "/path", 10, std::bind(&PathFollowerNode::path_callback, this, std::placeholders::_1));

    override_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotMotionOverride>(
        "/robot_motion_override", 10, std::bind(&PathFollowerNode::override_callback, this, std::placeholders::_1));

    // Assina a geometria oficial do campo para bloquear entrada na área de pênalti dinamicamente
    geometry_sub_ = this->create_subscription<oxebots_interfaces::msg::SSLGeometryData>(
        "/field_geometry", rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&PathFollowerNode::geometry_callback, this, std::placeholders::_1));

    // Assina o estado global de jogo consolidado pelo referee_state_node (SafetyGate)
    game_state_sub_ = this->create_subscription<oxebots_interfaces::msg::GameState>(
        "/game_state", rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&PathFollowerNode::game_state_callback, this, std::placeholders::_1));

    // Assina o limite explícito de velocidade publicado pela folha SetMotionLimit (SafetyGate)
    motion_limit_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/robot_motion_limit", rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&PathFollowerNode::motion_limit_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&PathFollowerNode::calculate_and_move, this));
    
    RCLCPP_INFO(this->get_logger(), "Seguidor de Caminho iniciado para o robô %d ouvindo %s/path", robot_id_, robot_prefix.c_str());
}

void PathFollowerNode::geometry_callback(const oxebots_interfaces::msg::SSLGeometryData::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_geometry_ = msg;
}

void PathFollowerNode::game_state_callback(const oxebots_interfaces::msg::GameState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_game_state_ = msg;
}

void PathFollowerNode::motion_limit_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (!msg || !std::isfinite(msg->data) || msg->data <= 0.0f) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    motion_limit_mps_ = msg->data;
}

void PathFollowerNode::clampVelocities(const movement::Coordinate& current_pos, float& vx, float& vy) {
    if (!last_geometry_) return;

    float field_length = static_cast<float>(last_geometry_->field.field_length);
    float penalty_depth = static_cast<float>(last_geometry_->field.penalty_area_depth);
    float penalty_width = static_cast<float>(last_geometry_->field.penalty_area_width);
    if (field_length <= 0.0f || penalty_depth <= 0.0f || penalty_width <= 0.0f) return;

    // Margem de segurança de 110mm (raio do robô 90mm + 20mm de tolerância)
    constexpr float kMarginMm = 110.0f;
    float area_x_limit = (field_length / 2.0f) - penalty_depth - kMarginMm;
    float area_y_limit = (penalty_width / 2.0f) + kMarginMm;

    float rx = current_pos.x;
    float ry = current_pos.y;

    bool block_positive = true;
    bool block_negative = true;

    if (robot_id_ == 0) {
        // O goleiro pode entrar na própria área, mas não na adversária
        if (rx > 0.0f) {
            block_positive = false;
        } else {
            block_negative = false;
        }
    }

    // 1. Bloqueia área positiva (X > area_x_limit, |Y| < area_y_limit)
    if (block_positive) {
        if (rx >= area_x_limit && std::abs(ry) < area_y_limit) {
            if (vx > 0.0f) {
                vx = 0.0f;
            }
            if (rx > (area_x_limit + 30.0f)) {
                vx = -0.3f;
            }
        }
        // Bloqueia entrada pelas laterais da área positiva (quando já está além do X da área e tenta entrar em Y)
        if (rx > (area_x_limit + 20.0f) && std::abs(ry) < (area_y_limit + 40.0f) && std::abs(ry) >= (area_y_limit - 30.0f)) {
            if (ry > 0.0f && vy < 0.0f) {
                vy = 0.0f;
            } else if (ry < 0.0f && vy > 0.0f) {
                vy = 0.0f;
            }
        }
    }

    // 2. Bloqueia área negativa (X < -area_x_limit, |Y| < area_y_limit)
    if (block_negative) {
        if (rx <= -area_x_limit && std::abs(ry) < area_y_limit) {
            if (vx < 0.0f) {
                vx = 0.0f;
            }
            if (rx < -(area_x_limit + 30.0f)) {
                vx = 0.3f;
            }
        }
        // Bloqueia entrada pelas laterais da área negativa
        if (rx < -(area_x_limit + 20.0f) && std::abs(ry) < (area_y_limit + 40.0f) && std::abs(ry) >= (area_y_limit - 30.0f)) {
            if (ry > 0.0f && vy < 0.0f) {
                vy = 0.0f;
            } else if (ry < 0.0f && vy > 0.0f) {
                vy = 0.0f;
            }
        }
    }
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
    target_planner_type_ = msg->planner_type;
    disable_arrival_latch_ = msg->disable_arrival_latch;
    target_max_speed_ = msg->max_speed;

    // SafetyGate: renova o watchdog por nova intenção vinda da árvore
    last_intent_time_ = std::chrono::steady_clock::now();
    has_received_intent_ = true;

    // Se havia um MODE_HALT segurado em override, a chegada de um novo goal libera explicitamente o robô
    if (last_override_msg_ && last_override_msg_->mode == oxebots_interfaces::msg::RobotMotionOverride::MODE_HALT) {
        last_override_msg_.reset();
    }
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

    // SafetyGate: renova o watchdog por nova intenção vinda da árvore
    last_intent_time_ = last_override_time_;
    has_received_intent_ = true;
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
    if (last_override_msg_) {
        if (last_override_msg_->mode == oxebots_interfaces::msg::RobotMotionOverride::MODE_HALT) {
            // MODE_HALT não expira por TTL de 200 ms (elimina a janela de 300 ms descrita no Cap. 17.5)
            effective_mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_HALT;
        } else if ((std::chrono::steady_clock::now() - last_override_time_) < kOverrideTtl) {
            effective_mode = last_override_msg_->mode;
        }
    }

    // Diagnóstico temporário: avisa sempre que o robô encostar (ou quase) na bola FORA do chute
    // intencional (MODE_KICK) — raio do robô (~75mm) + raio da bola (~22mm) ~ 97mm de contato
    // real. Ajuda a identificar em qual fase/estado exatamente a colisão relatada (bola atrás,
    // robô não desvia) está acontecendo, já que não há nenhum outro marcador de contato no log.
    if (effective_mode != oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK) {
        double ball_dist = std::hypot(last_game_data_->ball.x - current_pos.x, last_game_data_->ball.y - current_pos.y);
        if (ball_dist < 100.0) {
            RCLCPP_WARN(this->get_logger(),
                "Robô %d PERTO/ENCOSTANDO na bola fora do chute: dist=%.1fmm modo=%d heading=%.1fdeg "
                "robot(%.1f,%.1f) ball(%.1f,%.1f)",
                robot_id_, ball_dist, effective_mode, current_pos.orientation * 180.0 / M_PI,
                current_pos.x, current_pos.y, last_game_data_->ball.x, last_game_data_->ball.y);
        }
    }

    if (effective_mode == oxebots_interfaces::msg::RobotMotionOverride::MODE_HALT) {
        publishHalt();
        return;
    }
    if (effective_mode == oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK) {
        runKick(current_pos);
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
                    const auto& q = pose_stamped.pose.orientation;
                    float qw = std::atan2(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
                    target_pt = {px, py, qw};
                    target_found = true;
                    break;
                }
            }
            if (!target_found) {
                const auto& q = last_path_->poses.back().pose.orientation;
                float qw = std::atan2(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
                target_pt = {
                    (float)last_path_->poses.back().pose.position.x * 1000.0f,
                    (float)last_path_->poses.back().pose.position.y * 1000.0f,
                    qw
                };
                target_found = true;
            }
        } else if (last_path_ && last_path_->poses.empty()) {
            // Rota vazia recebida: parar o robô via SafetyGate
            oxebots_interfaces::msg::RobotCmdData cmd_data;
            cmd_data.id = robot_id_;
            cmd_data.x_velocity = 0.0f;
            cmd_data.y_velocity = 0.0f;
            cmd_data.angular_velocity = 0.0f;
            publishCommand(cmd_data, oxebots_interfaces::msg::RobotMotionOverride::MODE_NONE, false);
            return;
        }

    if (!target_found) {
        auto now = std::chrono::steady_clock::now();
        bool watchdog_expired = !has_received_intent_ || (now - last_intent_time_) > kWatchdogTimeout;
        bool is_halt = last_game_state_ && (
            last_game_state_->phase == oxebots_interfaces::msg::GameState::PHASE_HALT ||
            last_game_state_->phase == oxebots_interfaces::msg::GameState::PHASE_TIMEOUT);
        if (watchdog_expired || is_halt) {
            publishHalt();
        }
        return;
    }

    oxebots_interfaces::msg::RobotCmdData cmd_data;
    cmd_data.id = robot_id_;

    // --- Controle Linear (distância ao alvo, calculada antes do controle angular) ---
    double dist_to_target = std::hypot(target_pt.x - current_pos.x, target_pt.y - current_pos.y);

    // Se tivermos um alvo final, checamos se chegamos nele
    double check_dist = target_goal_.has_value() ?
        std::hypot(target_goal_->x - current_pos.x, target_goal_->y - current_pos.y) : dist_to_target;

    // Abaixo desta distância o alvo é considerado "alcançado": zera velocidade e libera o
    // controle angular (ver disable_arrival_latch_ abaixo). Precisa ficar abaixo de qualquer
    // tolerância usada nas árvores de comportamento.
    constexpr double kArrivedDistanceMm = 10.0;
    // Alvos publicados com tolerance<=0 (GoToPoint "nunca chega formalmente", ex: pontos de carga
    // deliberadamente além da bola) nunca travam aqui mesmo cruzando os 10mm — ver RobotGoal.msg.
    bool arrived = !disable_arrival_latch_ && (check_dist < kArrivedDistanceMm || dist_to_target < 0.001);

    // --- Controle Angular ---
    // Ao "chegar", este nó solta a autoridade angular (target_w_.reset()) para não brigar pelo
    // mesmo tópico /robot_commands com quem assumir a rotação em seguida.
    double angle_err = 0.0;
    bool has_target_w = target_w_.has_value();
    double desired_w = 0.0;
    bool should_control_w = false;

    if (arrived) {
        cmd_data.angular_velocity = 0.0;
        target_w_.reset();
        has_target_w = false;
    } else if (has_target_w) {
        desired_w = *target_w_;
        should_control_w = true;
    } else if (target_found) {
        desired_w = target_pt.orientation;
        should_control_w = true;
    }

    if (should_control_w) {
        angle_err = normalizeAngle(desired_w - current_pos.orientation);
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

        // Prioriza alinhar antes de avançar, mas só perto do alvo final: erro angular alto reduz
        // a velocidade linear (gira mais no lugar), alinhado mantém velocidade cheia. Evita que o
        // robô empurre/bata na bola fora de posição enquanto ainda gira para encará-la. Longe do
        // alvo, corre em velocidade plena sem esse freio.
        constexpr double kAlignmentPriorityRangeMm = 100.0;
        if (has_target_w && check_dist < kAlignmentPriorityRangeMm) {
            double alignment_factor = std::max(0.0, std::cos(angle_err));
            speed *= alignment_factor;
        }

        cmd_data.x_velocity = vx * speed;
        cmd_data.y_velocity = vy * speed;
    }

    // Repulsão de contato — só na Fase 1 (D*), onde qualquer contato com a bola é acidental (na
    // Fase 2/straight_line, chegar perto é o objetivo). Age no mesmo ciclo de 50ms em que o
    // contato é detectado, sem esperar o roundtrip árvore→D*→path. Histerese (entra a 100mm, só
    // desarma a 130mm) evita flapping na borda; complementa o ball_contact_threshold_mm do
    // GoToPoint, que força um replanejamento completo se o contato persistir.
    if (last_game_data_ && !arrived && target_planner_type_ == oxebots_interfaces::msg::RobotGoal::PLANNER_DSTAR) {
        double bdx = current_pos.x - last_game_data_->ball.x;
        double bdy = current_pos.y - last_game_data_->ball.y;
        double ball_dist = std::hypot(bdx, bdy);
        constexpr double kContactEnterMm = 100.0;
        constexpr double kContactExitMm = 130.0;

        if (contact_repulsion_active_) {
            if (ball_dist > kContactExitMm) contact_repulsion_active_ = false;
        } else if (ball_dist < kContactEnterMm) {
            contact_repulsion_active_ = true;
        }

        if (contact_repulsion_active_ && ball_dist > 1e-3) {
            double max_lin = this->get_parameter("max_linear_speed").as_double();
            double repel_speed = max_lin * std::clamp(1.0 - ball_dist / kContactExitMm, 0.0, 1.0);
            cmd_data.x_velocity = (bdx / ball_dist) * repel_speed;
            cmd_data.y_velocity = (bdy / ball_dist) * repel_speed;
            cmd_data.angular_velocity = 0.0;
        }
    }

    // Trava de segurança de velocidade para impedir entrada na área de pênalti (baseada em /field_geometry)
    clampVelocities(current_pos, cmd_data.x_velocity, cmd_data.y_velocity);

    publishCommand(cmd_data, oxebots_interfaces::msg::RobotMotionOverride::MODE_NONE, false);
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
    oxebots_interfaces::msg::RobotCmdData cmd_data;
    cmd_data.id = robot_id_;
    cmd_data.x_velocity = 0.0f;
    cmd_data.y_velocity = 0.0f;
    cmd_data.angular_velocity = 0.0f;
    cmd_data.kick_speed = 0.0f;
    publishCommand(cmd_data, oxebots_interfaces::msg::RobotMotionOverride::MODE_HALT, false);
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

    oxebots_interfaces::msg::RobotCmdData cmd_data;
    cmd_data.id = robot_id_;
    cmd_data.x_velocity = 0.0f;
    cmd_data.y_velocity = 0.0f;
    cmd_data.kick_speed = 0.0f;
    if (aligned) {
        cmd_data.angular_velocity = 0.0f;
    } else {
        double offset_rad = offset_deg * M_PI / 180.0;
        cmd_data.angular_velocity = std::clamp(
            offset_rad * last_override_msg_->p_gain,
            -last_override_msg_->max_angular_speed, last_override_msg_->max_angular_speed);
    }

    publishCommand(cmd_data, mode, aligned);

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
}

// Máquina de estados do chute, portada de KickBallNode. Enquanto MODE_KICK estiver ativo:
// dispara (avanço + kick_speed) por kKickFireWindow, incrementa kick_fire_count_ ao terminar a
// janela, e então segura kKickCooldown antes de aceitar outro pedido — sem isso, um pedido de
// chute que continue chegando (o leaf reenvia a cada tick enquanto RUNNING) dispararia de novo
// instantaneamente, virando uma "metralhadora" que nunca solta a bola direito.
void PathFollowerNode::runKick(const movement::Coordinate& current_pos) {
    auto now = std::chrono::steady_clock::now();

    if (kick_phase_ == KickPhase::IDLE && now >= kick_cooldown_until_) {
        kick_phase_ = KickPhase::FIRING;
        kick_start_time_ = now;
    }

    if (kick_phase_ == KickPhase::FIRING) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - kick_start_time_);
        if (elapsed < kKickFireWindow) {
            oxebots_interfaces::msg::RobotCmdData cmd_data;
            cmd_data.id = robot_id_;
            cmd_data.kick_speed = last_override_msg_->kick_speed;
            // Avançar enquanto chuta, para garantir contato — na direção que o robô está
            // REALMENTE olhando (current_pos.orientation), não em +X global fixo. RobotCmdData
            // usa velocidades em referencial global (mesma convenção do resto deste arquivo, ver
            // vx/vy do seguimento de caminho acima), então +X fixo só empurra "para frente" por
            // coincidência quando o robô já está de frente pro +X — ou seja, só funciona pro time
            // que ataca em direção a +X. Para o time que ataca em -X (opponent_goal_x negativo),
            // empurrava para TRÁS do chute, cancelando o próprio avanço — o chute "completava"
            // (kick_fire_count subia, timer terminava) mas a bola nunca se movia nem um
            // milímetro, mesmo com contato próximo (confirmado em teste: deslocamento=0.0mm em
            // toda tentativa, exclusivo do time amarelo).
            constexpr double kKickAdvanceSpeed = 0.5;
            cmd_data.x_velocity = kKickAdvanceSpeed * std::cos(current_pos.orientation);
            cmd_data.y_velocity = kKickAdvanceSpeed * std::sin(current_pos.orientation);
            cmd_data.angular_velocity = 0.0f;
            clampVelocities(current_pos, cmd_data.x_velocity, cmd_data.y_velocity);
            publishCommand(cmd_data, oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK, false);
            return;
        }

        kick_phase_ = KickPhase::IDLE;
        kick_cooldown_until_ = now + kKickCooldown;
        kick_fire_count_++;
    }

    // Fora da janela de disparo (aguardando pedido ou em cooldown): mantém o robô parado.
    oxebots_interfaces::msg::RobotCmdData cmd_data;
    cmd_data.id = robot_id_;
    cmd_data.x_velocity = 0.0f;
    cmd_data.y_velocity = 0.0f;
    cmd_data.angular_velocity = 0.0f;
    cmd_data.kick_speed = 0.0f;
    publishCommand(cmd_data, oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK, false);
}

void PathFollowerNode::applySafetyGate(oxebots_interfaces::msg::RobotCmdData& cmd, uint8_t& active_mode) {
    auto now = std::chrono::steady_clock::now();

    // 1. Regra 3 (Watchdog 500 ms - Capítulo 17.5 e SC-26):
    // Se o controlador não recebe um objetivo (/robot_goal) nem um override (/robot_motion_override)
    // há mais de 500 ms, zera os comandos e informa MODE_HALT no status. Protege contra árvore travada/muda.
    bool watchdog_expired = !has_received_intent_ || (now - last_intent_time_) > kWatchdogTimeout;
    if (watchdog_expired) {
        cmd.x_velocity = 0.0f;
        cmd.y_velocity = 0.0f;
        cmd.angular_velocity = 0.0f;
        cmd.kick_speed = 0.0f;
        active_mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_HALT;
    }

    // 2. Regra 1 (HALT zera a velocidade - Capítulo 17.5 e SC-02):
    // Se o GameState indica HALT (ou TIMEOUT), o comando é zero, mesmo que um override ou rota digam o contrário.
    if (last_game_state_) {
        if (last_game_state_->phase == oxebots_interfaces::msg::GameState::PHASE_HALT ||
            last_game_state_->phase == oxebots_interfaces::msg::GameState::PHASE_TIMEOUT) {
            cmd.x_velocity = 0.0f;
            cmd.y_velocity = 0.0f;
            cmd.angular_velocity = 0.0f;
            cmd.kick_speed = 0.0f;
            active_mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_HALT;
        }
    }

    // 3. Regra 2 (Teto de velocidade por fase e por objetivo - Capítulo 17.5, M03, SC-03 e R-27):
    // O módulo do vetor (vx, vy) é limitado a speed_limit_mps do GameState (0,75 m/s em parada, 1,5 m/s em jogo)
    // e também ao max_speed opcional pedido pelo GoToPoint no último RobotGoal.
    float speed_cap = static_cast<float>(this->get_parameter("max_linear_speed").as_double());
    if (last_game_state_ && last_game_state_->speed_limit_mps > 0.0f) {
        speed_cap = std::min(speed_cap, last_game_state_->speed_limit_mps);
    }
    if (motion_limit_mps_ > 0.0f) {
        speed_cap = std::min(speed_cap, motion_limit_mps_);
    }
    if (target_max_speed_ > 0.0f) {
        speed_cap = std::min(speed_cap, target_max_speed_);
    }
    float speed = std::hypot(cmd.x_velocity, cmd.y_velocity);
    if (speed > speed_cap && speed > 1e-4f) {
        float scale = speed_cap / speed;
        cmd.x_velocity *= scale;
        cmd.y_velocity *= scale;
    }

    // 4. Regra 4 (Limite do chute - Capítulo 17.5 e SC-24):
    // kick_speed é limitado a 3,0 m/s para qualquer origem, inclusive folhas legadas que esqueçam de limitar.
    cmd.kick_speed = std::clamp(cmd.kick_speed, 0.0f, 3.0f);
}

void PathFollowerNode::publishCommand(oxebots_interfaces::msg::RobotCmdData cmd_data, uint8_t active_mode, bool aligned) {
    applySafetyGate(cmd_data, active_mode);

    // Sanitiza contra valores não finitos (NaN/Inf)
    auto sanitize = [](float& val) {
        if (!std::isfinite(val)) val = 0.0f;
    };
    sanitize(cmd_data.x_velocity);
    sanitize(cmd_data.y_velocity);
    sanitize(cmd_data.angular_velocity);
    sanitize(cmd_data.kick_speed);

    auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
    cmd_msg->robots.push_back(cmd_data);
    cmd_vel_pub_->publish(std::move(cmd_msg));

    if (active_mode != oxebots_interfaces::msg::RobotMotionOverride::MODE_NONE) {
        publishStatus(active_mode, aligned);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
