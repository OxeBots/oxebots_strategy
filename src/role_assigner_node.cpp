#include <rclcpp/rclcpp.hpp>
#include <oxebots_interfaces/msg/game_data.hpp>
#include <oxebots_interfaces/msg/game_state.hpp>
#include <oxebots_interfaces/msg/role_assignment.hpp>
#include <cmath>
#include <algorithm>
#include <map>
#include <set>
#include <vector>

class RoleAssignerNode : public rclcpp::Node
{
public:
  explicit RoleAssignerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("role_assigner_node", options)
  {
    // Parâmetros configuráveis
    this->declare_parameter<int>("goalkeeper_id", 0);
    this->declare_parameter<double>("w1", 2.0);           // Peso para distância (Aumentado para priorizar quem está perto)
    this->declare_parameter<double>("w2", 1.0);           // Peso para alinhamento cinético
    this->declare_parameter<double>("hysteresis", 2.5);   // Vantagem de custo necessária para roubar o papel
    this->declare_parameter<double>("time_hysteresis", 2.0); // Tempo mínimo (s) entre trocas
    this->declare_parameter<bool>("is_yellow_team", false);
    // Velocidade (m/s) da bola acima da qual ela é considerada "em voo" (chute): os papéis ficam
    // congelados até ela desacelerar. 0 desliga o congelamento.
    this->declare_parameter<double>("ball_flight_speed_mps", 1.0);
    // Distância (mm) do candidato à bola abaixo da qual ele já recepcionou a bola: o congelamento
    // acaba e a troca de papel sai na hora, sem esperar a bola desacelerar.
    this->declare_parameter<double>("receive_distance_mm", 300.0);

    goalkeeper_id_ = this->get_parameter("goalkeeper_id").as_int();
    w1_ = this->get_parameter("w1").as_double();
    w2_ = this->get_parameter("w2").as_double();
    hysteresis_ = this->get_parameter("hysteresis").as_double();
    time_hysteresis_ = this->get_parameter("time_hysteresis").as_double();
    ball_flight_speed_mps_ = this->get_parameter("ball_flight_speed_mps").as_double();
    receive_distance_mm_ = this->get_parameter("receive_distance_mm").as_double();
    
    last_role_change_time_ = this->now();
    auto role_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    role_pub_ = this->create_publisher<oxebots_interfaces::msg::RoleAssignment>("/role_assignment", role_qos);
    game_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
      "game_data", 10, std::bind(&RoleAssignerNode::game_callback, this, std::placeholders::_1));
    game_state_sub_ = this->create_subscription<oxebots_interfaces::msg::GameState>("/game_state", role_qos, std::bind(&RoleAssignerNode::game_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Role Assigner ZJUNlict iniciado.");
    RCLCPP_INFO(this->get_logger(), "Goleiro ID: %d, w1: %.2f, w2: %.2f, Amarelo: %s", 
                goalkeeper_id_, w1_, w2_, this->get_parameter("is_yellow_team").as_bool() ? "Sim" : "Não");
  }

  void processGameState(const oxebots_interfaces::msg::GameState::SharedPtr msg)
  {
    game_state_callback(msg);
  }

  void processGameData(const oxebots_interfaces::msg::GameData::SharedPtr msg)
  {
    game_callback(msg);
  }

  oxebots_interfaces::msg::RoleAssignment getLastPublishedRoles() const
  {
    return last_published_roles_;
  }

private:
  struct Vector2D {
    double x, y;
    double norm() const { return std::sqrt(x * x + y * y); }
  };
  void game_state_callback(const oxebots_interfaces::msg::GameState::SharedPtr msg) {
    last_game_state_ = msg;
  }
  void game_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
  {
    auto now = this->now();
    double dt = 0.0;
    if (last_time_.nanoseconds() != 0) {
      dt = (now - last_time_).seconds();
    }
    last_time_ = now;

    if (dt <= 0.0001) dt = 0.016; // Fallback para ~60Hz e proteção contra dt zero
    constexpr uint32_t kNoRobot = oxebots_interfaces::msg::RoleAssignment::NO_ROBOT;

    uint8_t phase = oxebots_interfaces::msg::GameState::PHASE_RUN;
    int desired_gk_id = goalkeeper_id_;
    bool is_restart_phase = false;

    if (last_game_state_) {
      phase = last_game_state_->phase;
      // Se o GC estiver conectado e ativo, usa o goleiro do GC; caso contrário, faz fallback para o parâmetro
      if (!last_game_state_->stale) {
        desired_gk_id = static_cast<int>(last_game_state_->our_goalkeeper_id);
      } else {
        desired_gk_id = goalkeeper_id_;
      }

      const bool restart_phase_type =
        (phase == oxebots_interfaces::msg::GameState::PHASE_PREPARE_KICKOFF) ||
        (phase == oxebots_interfaces::msg::GameState::PHASE_KICKOFF) ||
        (phase == oxebots_interfaces::msg::GameState::PHASE_PREPARE_PENALTY) ||
        (phase == oxebots_interfaces::msg::GameState::PHASE_PENALTY) ||
        (phase == oxebots_interfaces::msg::GameState::PHASE_FREE_KICK) ||
        (phase == oxebots_interfaces::msg::GameState::PHASE_BALL_PLACEMENT);

      // O cobrador só deve ser travado se o lance for a nosso favor
      is_restart_phase = restart_phase_type && last_game_state_->we_favored;
    }

    // 1. Estimar velocidades (opcional, mas melhora o alinhamento cinético)
    Vector2D ball_pos = {msg->ball.x, msg->ball.y};

    // Velocidade da bola (m/s) para detectar bola em voo (chute). O game_data repete a MESMA posição
    // em ~60% das mensagens, então só se atualiza quando a posição mudou, com o tempo desde a última
    // posição distinta (derivar a cada mensagem alternava 0 e o dobro e o "em voo" piscava no meio
    // do chute). Parada por mais de 0,1 s -> 0; salto > 8 m/s (teletransporte) -> reancora.
    if (!has_last_ball_) {
      last_ball_pos_ = ball_pos;
      last_ball_change_ = now;
      has_last_ball_ = true;
    } else {
      const double since_change = (now - last_ball_change_).seconds();
      if (ball_pos.x != last_ball_pos_.x || ball_pos.y != last_ball_pos_.y) {
        if (since_change > 0.008) {
          const double v = std::hypot(ball_pos.x - last_ball_pos_.x, ball_pos.y - last_ball_pos_.y) /
                           since_change / 1000.0;
          ball_speed_mps_ = (v > 8.0) ? 0.0 : 0.5 * v + 0.5 * ball_speed_mps_;
          last_ball_pos_ = ball_pos;
          last_ball_change_ = now;
        }
      } else if (since_change > 0.10) {
        ball_speed_mps_ = 0.0;
      }
    }
    if (ball_flight_speed_mps_ > 0.0) {
      if (!ball_in_flight_ && ball_speed_mps_ > ball_flight_speed_mps_) {
        ball_in_flight_ = true;
      } else if (ball_in_flight_ && ball_speed_mps_ < 0.6 * ball_flight_speed_mps_) {
        ball_in_flight_ = false;
      }
    } else {
      ball_in_flight_ = false;
    }

    std::vector<uint32_t> active_ids;
    std::vector<uint32_t> eligible_ids;
    std::map<uint32_t, double> costs;
    std::map<uint32_t, double> ball_dists_mm;
    uint32_t active_goalkeeper_id = kNoRobot;
    for (const auto & robot : msg->robots.allies) {
      uint32_t id = robot.id;
      active_ids.push_back(id);
      if (static_cast<int>(id) == desired_gk_id) {
        active_goalkeeper_id = id;
        continue;
      }

      Vector2D r_pos = {robot.x, robot.y};
      
      // Estimação simples de velocidade do robô
      Vector2D r_vel = {0, 0};
      if (last_robot_pos_.count(id)) {
        r_vel.x = (r_pos.x - last_robot_pos_[id].x) / dt;
        r_vel.y = (r_pos.y - last_robot_pos_[id].y) / dt;
      }
      last_robot_pos_[id] = r_pos;

      // Cálculo de r (distância em metros)
      double dist_mm = std::sqrt(std::pow(r_pos.x - ball_pos.x, 2) + std::pow(r_pos.y - ball_pos.y, 2));
      double r = dist_mm / 1000.0; // r em metros
      ball_dists_mm[id] = dist_mm;

      // Cálculo de theta (alinhamento cinético)
      // Vetor do robô para a bola
      Vector2D to_ball = {ball_pos.x - r_pos.x, ball_pos.y - r_pos.y};
      double to_ball_norm = to_ball.norm();

      // Vetor direcional do robô
      // Se o robô estiver se movendo, usamos o vetor velocidade, senão usamos a orientação do chassi
      Vector2D r_dir;
      double vel_norm = r_vel.norm();
      if (vel_norm > 50.0) { // Se movendo a mais de 50mm/s
        r_dir = {r_vel.x / vel_norm, r_vel.y / vel_norm};
      } else {
        r_dir = {std::cos(robot.orientation), std::sin(robot.orientation)};
      }

      double cos_theta = 0.0;
      if (to_ball_norm > 1.0) { // Evitar divisão por zero se estiver em cima da bola
        cos_theta = (r_dir.x * to_ball.x + r_dir.y * to_ball.y) / to_ball_norm;
      } else {
        cos_theta = 1.0; // Já está na bola
      }

      // Custo simplificado: Distância linear (mais punitiva) e alinhamento cinético
      double cost = w1_ * r + w2_ * (1.0 - cos_theta);
      
      // Bônus de Adesão (Stickiness): Quem já é o atacante tem um "desconto" no custo
      if (!is_restart_phase && id == last_attacker_id_) {
          cost -= hysteresis_;
      }

      costs[id] = cost;
      eligible_ids.push_back(id);
    }

    // Ordena os robôs de linha elegíveis pelo menor custo
    std::sort(eligible_ids.begin(), eligible_ids.end(), [&](uint32_t a, uint32_t b) {
      return costs[a] < costs[b];
    });

    // P03: degradação por limite de robôs em campo (cartão amarelo ou vermelho: max_allowed_bots do
    // GC). O robô retirado sai de active_ids e dos papéis (o strategy_node o vê como "idle"), e fica
    // parado até um humano tirá-lo do campo; quando some da visão o limite passa a ser cumprido.
    // 0 = o GC não informou o limite (sem GameState, ou GameState sem o campo): sem retirada.
    const uint32_t max_allowed = last_game_state_ ? last_game_state_->max_allowed_bots : 0u;
    const std::set<uint32_t> retired = selectRetiredRobots(
      eligible_ids, active_ids.size(), max_allowed);
    if (!retired.empty()) {
      const auto is_retired = [&retired](uint32_t id) {return retired.count(id) > 0;};
      eligible_ids.erase(
        std::remove_if(eligible_ids.begin(), eligible_ids.end(), is_retired), eligible_ids.end());
      active_ids.erase(
        std::remove_if(active_ids.begin(), active_ids.end(), is_retired), active_ids.end());
    }

    uint32_t attacker_id = kNoRobot;
    uint32_t defender_id = kNoRobot;
    uint32_t kicker_id = kNoRobot;

    if (is_restart_phase) {
      // Verifica se o cobrador já travado continua ativo entre os elegíveis
      bool locked_still_eligible =
        (locked_kicker_id_ != kNoRobot) &&
        (std::find(eligible_ids.begin(), eligible_ids.end(), locked_kicker_id_) != eligible_ids.end());

      if (!locked_still_eligible) {
        locked_kicker_id_ = eligible_ids.empty() ? kNoRobot : eligible_ids[0];
      }

      kicker_id = locked_kicker_id_;
      attacker_id = locked_kicker_id_;

      // Defensor é o próximo robô elegível diferente do cobrador
      for (uint32_t cand : eligible_ids) {
        if (cand != attacker_id) {
          defender_id = cand;
          break;
        }
      }
    } else {
      // Fora de reinício: destrava o cobrador
      locked_kicker_id_ = kNoRobot;
      kicker_id = kNoRobot;

      if (eligible_ids.size() == 1) {
        attacker_id = eligible_ids[0];
        defender_id = kNoRobot;
      } else if (eligible_ids.size() >= 2) {
        uint32_t best_candidate = eligible_ids[0];
        uint32_t second_best = eligible_ids[1];
        double time_since_change = (now - last_role_change_time_).seconds();

        bool last_attacker_eligible =
          (last_attacker_id_ != kNoRobot) &&
          (std::find(eligible_ids.begin(), eligible_ids.end(), last_attacker_id_) != eligible_ids.end());

        if (last_attacker_eligible && best_candidate != last_attacker_id_) {
          // Quem vai assumir já está em cima da bola (recepcionou) e a bola quase parou: não há mais o que
          // proteger. A condição de velocidade é essencial: o defensor que marca o chutador fica a ~400 mm
          // dele, e a bola chutada passa a menos de receive_distance_mm dele a alta velocidade; sem ela a
          // troca saía no meio do chute e o defensor largava a linha bola-gol.
          const bool candidate_has_ball =
            ball_dists_mm[best_candidate] < receive_distance_mm_ &&
            ball_speed_mps_ < kReceiveMaxSpeedFrac * ball_flight_speed_mps_;
          const bool frozen = ball_in_flight_ && !candidate_has_ball;
          if (time_since_change < time_hysteresis_ || frozen) {
            if (frozen) {
              RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Bola em voo (%.2f m/s): papéis congelados (atacante=%u).", ball_speed_mps_, last_attacker_id_);
            }
            attacker_id = last_attacker_id_;
            defender_id = (attacker_id == eligible_ids[0]) ? eligible_ids[1] : eligible_ids[0];
          } else {
            attacker_id = best_candidate;
            defender_id = second_best;
            last_role_change_time_ = now;
            RCLCPP_INFO(this->get_logger(), "TROCA DE PAPEL: Novo Atacante = %u", attacker_id);
          }
        } else {
          attacker_id = best_candidate;
          defender_id = second_best;
          if (!last_attacker_eligible) last_role_change_time_ = now;
        }
      }
    }

    last_attacker_id_ = attacker_id;

    // Ordenação defensiva de active_ids para evitar saltos espúrios de epoch quando a ordem em allies variar
    std::sort(active_ids.begin(), active_ids.end());

    // Monta a mensagem RoleAssignment completa
    oxebots_interfaces::msg::RoleAssignment out_msg;
    out_msg.goalkeeper_id = active_goalkeeper_id;
    out_msg.attacker_id = attacker_id;
    out_msg.defender_id = defender_id;
    out_msg.kicker_id = kicker_id;
    out_msg.active_ids = active_ids;

    // Incrementa epoch sempre que houver mudança em qualquer papel ou lista de ativos
    bool roles_changed = !has_published_roles_ ||
      last_published_roles_.goalkeeper_id != out_msg.goalkeeper_id ||
      last_published_roles_.attacker_id != out_msg.attacker_id ||
      last_published_roles_.defender_id != out_msg.defender_id ||
      last_published_roles_.kicker_id != out_msg.kicker_id ||
      last_published_roles_.active_ids != out_msg.active_ids;

    if (roles_changed) {
      epoch_++;
      has_published_roles_ = true;
    }
    out_msg.epoch = epoch_;
    last_published_roles_ = out_msg;

    role_pub_->publish(out_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Roles [epoch=%u]: GK=%u, ATT=%u, DEF=%u, KICK=%u (ativos=%zu)",
      out_msg.epoch, out_msg.goalkeeper_id, out_msg.attacker_id,
      out_msg.defender_id, out_msg.kicker_id, out_msg.active_ids.size());
  }

  // P03: escolhe quais robôs retirar quando há mais robôs em campo que max_allowed_bots.
  //  - line_by_cost: robôs de linha visíveis (sem o goleiro), do melhor para o pior custo;
  //  - o goleiro nunca é retirado: com max_allowed >= 1 sempre sobra robô de linha para retirar
  //    (os de linha são visible_count - 1 ou mais, e se retira no máximo visible_count - 1);
  //  - a escolha é pegajosa: quem já estava retirado continua sendo o primeiro a sair, para a
  //    retirada não trocar de robô enquanto o humano o tira do campo.
  std::set<uint32_t> selectRetiredRobots(
    const std::vector<uint32_t> & line_by_cost, size_t visible_count, uint32_t max_allowed)
  {
    std::set<uint32_t> retire;
    if (max_allowed > 0 && visible_count > max_allowed) {
      size_t to_retire = visible_count - max_allowed;

      std::vector<uint32_t> order;
      for (auto it = line_by_cost.rbegin(); it != line_by_cost.rend(); ++it) {
        if (retired_ids_.count(*it) > 0) {
          order.push_back(*it);
        }
      }
      for (auto it = line_by_cost.rbegin(); it != line_by_cost.rend(); ++it) {
        if (retired_ids_.count(*it) == 0) {
          order.push_back(*it);
        }
      }
      for (size_t i = 0; i < order.size() && retire.size() < to_retire; ++i) {
        retire.insert(order[i]);
      }
    }

    if (retire != retired_ids_) {
      if (retire.empty()) {
        RCLCPP_INFO(this->get_logger(),
          "Limite de robôs cumprido (max_allowed_bots=%u, em campo=%zu): nenhum robô retirado.",
          max_allowed, visible_count);
      } else {
        for (uint32_t id : retire) {
          RCLCPP_WARN(this->get_logger(),
            "max_allowed_bots=%u e há %zu robôs em campo: RETIRAR o robô %u do campo "
            "(fica parado, papel idle).", max_allowed, visible_count, id);
        }
      }
      retired_ids_ = retire;
    }
    return retire;
  }

  int goalkeeper_id_;
  std::set<uint32_t> retired_ids_;
  uint32_t locked_kicker_id_ = oxebots_interfaces::msg::RoleAssignment::NO_ROBOT;
  uint32_t epoch_ = 0;
  oxebots_interfaces::msg::RoleAssignment last_published_roles_;
  bool has_published_roles_ = false;
  double w1_, w2_, hysteresis_, time_hysteresis_;
  double ball_flight_speed_mps_ = 1.0;
  double receive_distance_mm_ = 300.0;
  static constexpr double kReceiveMaxSpeedFrac = 0.8;  // da velocidade de voo
  double ball_speed_mps_ = 0.0;
  bool ball_in_flight_ = false;
  bool has_last_ball_ = false;
  rclcpp::Time last_ball_change_;
  uint32_t last_attacker_id_ = oxebots_interfaces::msg::RoleAssignment::NO_ROBOT;
  rclcpp::Time last_time_, last_role_change_time_;
  oxebots_interfaces::msg::GameState::SharedPtr last_game_state_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameState>::SharedPtr game_state_sub_;
  Vector2D last_ball_pos_;
  std::map<uint32_t, Vector2D> last_robot_pos_;
  rclcpp::Publisher<oxebots_interfaces::msg::RoleAssignment>::SharedPtr role_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_sub_;
};

#ifndef ROLE_ASSIGNER_NODE_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoleAssignerNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
