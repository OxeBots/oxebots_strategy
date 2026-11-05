#include <rclcpp/rclcpp.hpp>
#include <oxebots_interfaces/msg/game_data.hpp>
#include <oxebots_interfaces/msg/game_state.hpp>
#include <oxebots_interfaces/msg/role_assignment.hpp>
#include <cmath>
#include <algorithm>
#include <map>
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

    goalkeeper_id_ = this->get_parameter("goalkeeper_id").as_int();
    w1_ = this->get_parameter("w1").as_double();
    w2_ = this->get_parameter("w2").as_double();
    hysteresis_ = this->get_parameter("hysteresis").as_double();
    time_hysteresis_ = this->get_parameter("time_hysteresis").as_double();
    
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
      desired_gk_id = static_cast<int>(last_game_state_->our_goalkeeper_id);
      is_restart_phase = 
      (phase == oxebots_interfaces::msg::GameState::PHASE_PREPARE_KICKOFF) ||
      (phase == oxebots_interfaces::msg::GameState::PHASE_KICKOFF) || 
      (phase == oxebots_interfaces::msg::GameState::PHASE_PREPARE_PENALTY) ||
      (phase == oxebots_interfaces::msg::GameState::PHASE_PENALTY) ||
      (phase == oxebots_interfaces::msg::GameState::PHASE_FREE_KICK) ||
      (phase == oxebots_interfaces::msg::GameState::PHASE_BALL_PLACEMENT);
    }
    // 1. Estimar velocidades (opcional, mas melhora o alinhamento cinético)
    Vector2D ball_pos = {msg->ball.x, msg->ball.y};
    last_ball_pos_ = ball_pos;

    std::vector<uint32_t> active_ids;
    std::vector<uint32_t> eligible_ids;
    std::map<uint32_t, double> costs;
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
          if (time_since_change < time_hysteresis_) {
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

  int goalkeeper_id_;
  uint32_t locked_kicker_id_ = oxebots_interfaces::msg::RoleAssignment::NO_ROBOT;
  uint32_t epoch_ = 0;
  oxebots_interfaces::msg::RoleAssignment last_published_roles_;
  bool has_published_roles_ = false;
  double w1_, w2_, hysteresis_, time_hysteresis_;
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
