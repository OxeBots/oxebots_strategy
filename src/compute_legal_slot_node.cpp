#include "oxebots_strategy/compute_legal_slot_node.hpp"
#include "oxebots_strategy/defender_nodes.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace oxebots_strategy
{

namespace
{

double normalizeAngle(double angle)
{
  if (!std::isfinite(angle)) return 0.0;
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

template <typename T>
bool getFromBlackboards(const BT::Blackboard::Ptr& local_bb,
                        BT::Blackboard* root_bb,
                        const std::string& key,
                        T& out)
{
  if (local_bb && local_bb->get<T>(key, out)) {
    return true;
  }
  if (root_bb && root_bb != local_bb.get() && root_bb->get<T>(key, out)) {
    return true;
  }
  return false;
}

void pushOutOfPenaltyArea(double& x, double& y,
                          double goal_x, double depth, double width,
                          double area_margin, double max_y)
{
  const double half_w = width / 2.0 + area_margin;
  if (std::abs(y) >= half_w) {
    return;
  }

  if (goal_x > 0.0) {
    const double x_limit = goal_x - depth - area_margin;
    if (x > x_limit) {
      const double dx = x - x_limit;
      const double target_y = (y >= 0.0) ? half_w : -half_w;
      const double dy = std::abs(target_y - y);
      if (dy < dx && std::abs(target_y) <= max_y) {
        y = target_y;
      } else {
        x = x_limit;
      }
    }
  } else {
    const double x_limit = goal_x + depth + area_margin;
    if (x < x_limit) {
      const double dx = x_limit - x;
      const double target_y = (y >= 0.0) ? half_w : -half_w;
      const double dy = std::abs(target_y - y);
      if (dy < dx && std::abs(target_y) <= max_y) {
        y = target_y;
      } else {
        x = x_limit;
      }
    }
  }
}

void clampToLegalField(double& x, double& y,
                       double own_goal_x,
                       double field_length, double field_width,
                       double penalty_depth, double penalty_width,
                       double margin,
                       bool is_goalkeeper)
{
  const double half_len = field_length / 2.0;
  const double half_wid = field_width / 2.0;
  const double max_x = std::max(0.0, half_len - margin);
  const double max_y = std::max(0.0, half_wid - margin);

  x = std::clamp(x, -max_x, max_x);
  y = std::clamp(y, -max_y, max_y);

  // Área adversária proibida para todos os robôs (R-21: >= 200 mm da área adversária)
  const double opp_goal_x = -own_goal_x;
  const double opp_area_margin = std::max(220.0, margin);
  pushOutOfPenaltyArea(x, y, opp_goal_x, penalty_depth, penalty_width, opp_area_margin, max_y);

  // Área própria proibida para quem não é goleiro
  if (!is_goalkeeper) {
    pushOutOfPenaltyArea(x, y, own_goal_x, penalty_depth, penalty_width, margin, max_y);
  }

  x = std::clamp(x, -max_x, max_x);
  y = std::clamp(y, -max_y, max_y);
}

}  // namespace

ComputeLegalSlotNode::ComputeLegalSlotNode(const std::string& name,
                                           const BT::NodeConfig& config,
                                           rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), node_(node)
{
  if (!node_->has_parameter("slot_margin_mm")) {
    node_->declare_parameter<double>("slot_margin_mm", 150.0);
  }
}

BT::PortsList ComputeLegalSlotNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("slot", "Nome do slot tático (retreat, stop_shadow, stop_def, "
                                       "kickoff_kicker, kickoff_def, "
                                       "penalty_kicker, penalty_behind, freekick_support, wall)"),
    BT::InputPort<unsigned int>("index", 0u, "Índice do robô no slot"),
    BT::OutputPort<double>("slot_x", "Coordenada X calculada (mm)"),
    BT::OutputPort<double>("slot_y", "Coordenada Y calculada (mm)"),
    BT::OutputPort<double>("slot_w", "Orientação calculada (rad)")
  };
}

BT::NodeStatus ComputeLegalSlotNode::tick()
{
  std::string slot;
  if (!getInput<std::string>("slot", slot) || slot.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "ComputeLegalSlot: porta obrigatoria 'slot' ausente ou vazia");
    return BT::NodeStatus::FAILURE;
  }

  unsigned int index = 0u;
  getInput<unsigned int>("index", index);

  double margin = 150.0;
  if (node_->has_parameter("slot_margin_mm")) {
    margin = std::max(0.0, node_->get_parameter("slot_margin_mm").as_double());
  }

  auto local_bb = config().blackboard;
  auto root_bb = local_bb ? local_bb->rootBlackboard() : nullptr;
  if (!local_bb) {
    RCLCPP_ERROR(node_->get_logger(), "ComputeLegalSlot: blackboard inexistente");
    return BT::NodeStatus::FAILURE;
  }

  // 1. Lado do campo: compatível com D03 (own_goal_x / side_sign) e esquema atual (my_goal_x)
  double own_goal_x = 0.0;
  if (!getFromBlackboards(local_bb, root_bb, "own_goal_x", own_goal_x) || std::abs(own_goal_x) < 1.0) {
    if (!getFromBlackboards(local_bb, root_bb, "my_goal_x", own_goal_x) || std::abs(own_goal_x) < 1.0) {
      int side_sign_bb = 0;
      if (getFromBlackboards(local_bb, root_bb, "side_sign", side_sign_bb) && side_sign_bb != 0) {
        double fl = 4500.0;
        getFromBlackboards(local_bb, root_bb, "field_length", fl);
        if (fl <= 1.0) fl = 4500.0;
        // Cap. 20: side_sign = +1 quando defendemos x < 0 (own_goal_x = -L/2)
        own_goal_x = (side_sign_bb > 0) ? -(fl / 2.0) : (fl / 2.0);
      } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "ComputeLegalSlot: sem own_goal_x/my_goal_x na blackboard");
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  // Convenção do Cap. 20:
  // own_goal_x < 0 => defendemos a esquerda (nossa metade tem x < 0, atacamos +x, side_sign = +1)
  // own_goal_x > 0 => defendemos a direita  (nossa metade tem x > 0, atacamos -x, side_sign = -1)
  int side_sign = (own_goal_x < 0.0) ? 1 : -1;
  int side_sign_bb = 0;
  if (getFromBlackboards(local_bb, root_bb, "side_sign", side_sign_bb) &&
      (side_sign_bb == 1 || side_sign_bb == -1)) {
    side_sign = side_sign_bb;
  }

  const double our_half_sign = -static_cast<double>(side_sign);  // sinal de own_goal_x
  const double attack_sign = static_cast<double>(side_sign);     // sentido rumo ao gol adversário
  const double opp_goal_x = -own_goal_x;

  // 2. Geometria do campo (com padrões SSL-EL caso alguma chave específica seja 0)
  double field_length = 0.0;
  if (!getFromBlackboards(local_bb, root_bb, "field_length", field_length) || field_length <= 1.0) {
    field_length = 2.0 * std::abs(own_goal_x);
  }
  double field_width = 0.0;
  if (!getFromBlackboards(local_bb, root_bb, "field_width", field_width) || field_width <= 1.0) {
    field_width = 3000.0;
  }
  double penalty_depth = 0.0;
  if (!getFromBlackboards(local_bb, root_bb, "penalty_area_depth", penalty_depth) || penalty_depth <= 1.0) {
    penalty_depth = 500.0;
  }
  double penalty_width = 0.0;
  if (!getFromBlackboards(local_bb, root_bb, "penalty_area_width", penalty_width) || penalty_width <= 1.0) {
    penalty_width = 1350.0;
  }
  double center_circle_radius = 0.0;
  if (!getFromBlackboards(local_bb, root_bb, "center_circle_radius", center_circle_radius) ||
      center_circle_radius <= 1.0) {
    center_circle_radius = 500.0;
  }

  // 3. Identidade de goleiro e robot_id
  uint32_t robot_id = 0;
  bool has_robot_id = getFromBlackboards(local_bb, root_bb, "robot_id", robot_id);
  if (!has_robot_id) {
    unsigned int rid_u = 0;
    if (getFromBlackboards(local_bb, root_bb, "robot_id", rid_u)) {
      robot_id = rid_u;
      has_robot_id = true;
    }
  }

  bool is_goalkeeper = false;
  if (!getFromBlackboards(local_bb, root_bb, "is_goalkeeper", is_goalkeeper)) {
    std::string role_str;
    if (getFromBlackboards(local_bb, root_bb, "role", role_str)) {
      is_goalkeeper = (role_str == "goalkeeper");
    } else if (has_robot_id && robot_id == 0) {
      is_goalkeeper = true;
    }
  }

  // 4. Posição da bola (Pose2D 'ball', pares 'ball_x'/'ball_y')
  double ball_x = 0.0;
  double ball_y = 0.0;
  bool has_ball = false;

  Pose2D ball_pose{0.0, 0.0, 0.0};
  double bx_bb = 0.0, by_bb = 0.0;
  const bool has_ball_pose = getFromBlackboards(local_bb, root_bb, "ball", ball_pose);
  const bool has_ball_xy = getFromBlackboards(local_bb, root_bb, "ball_x", bx_bb) &&
                           getFromBlackboards(local_bb, root_bb, "ball_y", by_bb);

  if (has_ball_xy && (std::abs(bx_bb) > 1e-6 || std::abs(by_bb) > 1e-6)) {
    ball_x = bx_bb;
    ball_y = by_bb;
    has_ball = true;
  } else if (has_ball_pose && (std::abs(ball_pose.x) > 1e-6 || std::abs(ball_pose.y) > 1e-6)) {
    ball_x = ball_pose.x;
    ball_y = ball_pose.y;
    has_ball = true;
  } else if (has_ball_xy) {
    ball_x = bx_bb;
    ball_y = by_bb;
    has_ball = true;
  } else if (has_ball_pose) {
    ball_x = ball_pose.x;
    ball_y = ball_pose.y;
    has_ball = true;
  }

  if (!has_ball) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "ComputeLegalSlot: posição da bola indisponível");
    return BT::NodeStatus::FAILURE;
  }

  // 5. Posição do robô (Pose2D 'robot_pose', pares 'robot_x'/'robot_y')
  double robot_x = 0.0;
  double robot_y = 0.0;
  bool has_robot_pose = false;

  Pose2D robot_pose{0.0, 0.0, 0.0};
  double rx_bb = 0.0, ry_bb = 0.0;
  const bool has_rp_struct = getFromBlackboards(local_bb, root_bb, "robot_pose", robot_pose);
  const bool has_rp_xy = getFromBlackboards(local_bb, root_bb, "robot_x", rx_bb) &&
                         getFromBlackboards(local_bb, root_bb, "robot_y", ry_bb);

  if (has_rp_struct && (std::abs(robot_pose.x) > 1e-6 || std::abs(robot_pose.y) > 1e-6)) {
    robot_x = robot_pose.x;
    robot_y = robot_pose.y;
    has_robot_pose = true;
  } else if (has_rp_xy && (std::abs(rx_bb) > 1e-6 || std::abs(ry_bb) > 1e-6)) {
    robot_x = rx_bb;
    robot_y = ry_bb;
    has_robot_pose = true;
  } else if (has_rp_struct) {
    robot_x = robot_pose.x;
    robot_y = robot_pose.y;
    has_robot_pose = true;
  } else if (has_rp_xy) {
    robot_x = rx_bb;
    robot_y = ry_bb;
    has_robot_pose = true;
  }

  // 6. Cálculo do slot solicitado
  double slot_x = 0.0;
  double slot_y = 0.0;
  double slot_w = 0.0;
  bool enforce_ball_clearance = false;
  const double min_stop_ball_dist = 500.0 + margin;  // 650 mm quando margin = 150 mm

  if (slot == "retreat") {
    if (!has_robot_pose) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "ComputeLegalSlot(retreat): pose do robô indisponível");
      return BT::NodeStatus::FAILURE;
    }

    const double dx = robot_x - ball_x;
    const double dy = robot_y - ball_y;
    const double dist = std::hypot(dx, dy);

    double dir_angle = 0.0;
    if (dist > 1e-3) {
      dir_angle = std::atan2(dy, dx);
    } else {
      // Se robô e bola coincidem, recua na direção do próprio gol
      dir_angle = std::atan2(0.0 - ball_y, own_goal_x - ball_x);
    }

    slot_x = ball_x + min_stop_ball_dist * std::cos(dir_angle);
    slot_y = ball_y + min_stop_ball_dist * std::sin(dir_angle);
    enforce_ball_clearance = true;
  } else if (slot == "stop_shadow") {
    // Atacante no STOP: posiciona-se atrás da bola na linha bola -> gol adversário a >= 700 mm
    const double gx = opp_goal_x - ball_x;
    const double gy = 0.0 - ball_y;
    const double g_norm = std::hypot(gx, gy);
    const double ux = (g_norm > 1e-3) ? (gx / g_norm) : attack_sign;
    const double uy = (g_norm > 1e-3) ? (gy / g_norm) : 0.0;
    const double shadow_dist = std::max(700.0, min_stop_ball_dist + 50.0);

    slot_x = ball_x - ux * shadow_dist;
    slot_y = ball_y - uy * shadow_dist;
    slot_w = std::atan2(0.0 - slot_y, opp_goal_x - slot_x);
    enforce_ball_clearance = true;
  } else if (slot == "stop_def") {
    // Defensor no STOP: acompanha a bola fechando a linha bola -> nosso gol a >= 800 mm
    const double dx = own_goal_x - ball_x;
    const double dy = 0.0 - ball_y;
    const double dist = std::hypot(dx, dy);
    const double ux = (dist > 1e-3) ? (dx / dist) : our_half_sign;
    const double uy = (dist > 1e-3) ? (dy / dist) : 0.0;
    const double px = -uy;
    const double py = ux;

    const double def_dist = std::max(800.0, min_stop_ball_dist + 150.0);
    const double lat = (index == 0u)
                           ? 0.0
                           : ((index % 2u == 1u) ? 220.0 : -220.0) *
                                 static_cast<double>((index + 1u) / 2u);

    slot_x = ball_x + ux * def_dist + px * lat;
    slot_y = ball_y + uy * def_dist + py * lat;
    enforce_ball_clearance = true;
  } else if (slot == "kickoff_kicker") {
    // Dentro do círculo central, atrás da bola (no nosso lado), voltado ao gol adversário
    const double kicker_dist = std::min(280.0, std::max(180.0, center_circle_radius - margin));
    slot_x = our_half_sign * kicker_dist;
    slot_y = 0.0;
    slot_w = std::atan2(0.0 - slot_y, opp_goal_x - slot_x);
  } else if (slot == "kickoff_def") {
    // Na própria metade, fora do círculo central (>= raio + margem), escalonado pelo index
    const double def_x_dist = std::max(900.0, center_circle_radius + margin + 200.0);
    slot_x = our_half_sign * def_x_dist;
    if (index == 0u) {
      slot_y = attack_sign * 520.0;
    } else if (index == 1u) {
      slot_y = -attack_sign * 520.0;
    } else {
      const double sign_y = (index % 2u == 0u) ? 1.0 : -1.0;
      slot_y = sign_y * (300.0 + static_cast<double>(index / 2u) * 350.0);
    }
    slot_w = std::atan2(ball_y - slot_y, ball_x - slot_x);
  } else if (slot == "penalty_kicker") {
    // Atrás da bola na linha bola -> gol adversário, sem tocá-la
    const double gx = opp_goal_x - ball_x;
    const double gy = 0.0 - ball_y;
    const double g_norm = std::hypot(gx, gy);
    const double ux = (g_norm > 1e-3) ? (gx / g_norm) : attack_sign;
    const double uy = (g_norm > 1e-3) ? (gy / g_norm) : 0.0;
    const double pk_dist = std::max(300.0, margin + 150.0);

    slot_x = ball_x - ux * pk_dist;
    slot_y = ball_y - uy * pk_dist;
    slot_w = std::atan2(0.0 - slot_y, opp_goal_x - slot_x);
  } else if (slot == "penalty_behind") {
    // >= 1 m atrás da bola (com margem), no sentido oposto ao gol atacado no pênalti
    const double behind_sign =
        (std::abs(ball_x) > 100.0) ? ((ball_x > 0.0) ? -1.0 : 1.0) : our_half_sign;
    slot_x = ball_x + behind_sign * (1000.0 + margin);

    const double sign_y = (index % 2u == 0u) ? 1.0 : -1.0;
    slot_y = sign_y * (500.0 + static_cast<double>(index / 2u) * 400.0);
    slot_w = std::atan2(ball_y - slot_y, ball_x - slot_x);
  } else if (slot == "freekick_support") {
    // Posição de apoio longe da bola (>= 0,5 m + margem) e >= 0,2 m da área adversária
    double ref_x = ball_x;
    double ref_y = ball_y;
    Pose2D des_pose{0.0, 0.0, 0.0};
    double dx_bb = 0.0, dy_bb = 0.0;
    if (getFromBlackboards(local_bb, root_bb, "designated", des_pose) &&
        (std::abs(des_pose.x) > 1e-3 || std::abs(des_pose.y) > 1e-3)) {
      ref_x = des_pose.x;
      ref_y = des_pose.y;
    } else if (getFromBlackboards(local_bb, root_bb, "designated_x", dx_bb) &&
               getFromBlackboards(local_bb, root_bb, "designated_y", dy_bb) &&
               (std::abs(dx_bb) > 1e-3 || std::abs(dy_bb) > 1e-3)) {
      ref_x = dx_bb;
      ref_y = dy_bb;
    }

    slot_x = ref_x + our_half_sign * (650.0 + margin);
    double side_y = (ref_y >= 0.0) ? -600.0 : 600.0;
    if (index % 2u == 1u) {
      side_y = -side_y;
    }
    slot_y = side_y;
    enforce_ball_clearance = true;
  } else if (slot == "wall") {
    // Barreira entre a bola e o nosso gol (own_goal_x, 0), a >= 0,5 m + margem da bola
    const double dx = own_goal_x - ball_x;
    const double dy = 0.0 - ball_y;
    const double dist = std::hypot(dx, dy);
    const double ux = (dist > 1e-3) ? (dx / dist) : our_half_sign;
    const double uy = (dist > 1e-3) ? (dy / dist) : 0.0;
    const double px = -uy;
    const double py = ux;

    const double wall_dist = std::max(min_stop_ball_dist, std::min(900.0, dist * 0.4));
    const double lat = (index == 0u)
                           ? 0.0
                           : ((index % 2u == 1u) ? 220.0 : -220.0) *
                                 static_cast<double>((index + 1u) / 2u);

    slot_x = ball_x + ux * wall_dist + px * lat;
    slot_y = ball_y + uy * wall_dist + py * lat;
    enforce_ball_clearance = true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "ComputeLegalSlot: slot desconhecido '%s'", slot.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 7. Filtro final: mantém dentro do campo e fora das áreas proibidas
  const double desired_x = slot_x;
  const double desired_y = slot_y;
  clampToLegalField(slot_x, slot_y, own_goal_x, field_length, field_width,
                    penalty_depth, penalty_width, margin, is_goalkeeper);

  // Se o filtro final aproximou o ponto da bola abaixo do mínimo regulamentar (ex: bola perto da
  // lateral ou da área no STOP/wall/freekick_support), busca o ângulo válido mais próximo ao redor da bola.
  if (enforce_ball_clearance &&
      std::hypot(slot_x - ball_x, slot_y - ball_y) < (min_stop_ball_dist - 1.0)) {
    const double base_ang = std::atan2(desired_y - ball_y, desired_x - ball_x);
    double best_x = slot_x;
    double best_y = slot_y;
    double best_cost = std::numeric_limits<double>::infinity();
    const double ref_x = has_robot_pose ? robot_x : desired_x;
    const double ref_y = has_robot_pose ? robot_y : desired_y;

    for (int step = 1; step <= 12; ++step) {
      for (int sign : {1, -1}) {
        const double ang = base_ang + sign * step * (M_PI / 12.0);
        double cx = ball_x + min_stop_ball_dist * std::cos(ang);
        double cy = ball_y + min_stop_ball_dist * std::sin(ang);
        clampToLegalField(cx, cy, own_goal_x, field_length, field_width,
                          penalty_depth, penalty_width, margin, is_goalkeeper);
        if (std::hypot(cx - ball_x, cy - ball_y) >= 500.0 + std::min(margin, 100.0)) {
          const double cost = std::hypot(cx - ref_x, cy - ref_y);
          if (cost < best_cost) {
            best_cost = cost;
            best_x = cx;
            best_y = cy;
          }
        }
      }
      if (std::isfinite(best_cost)) {
        break;
      }
    }
    slot_x = best_x;
    slot_y = best_y;
  }

  if (slot == "retreat" || slot == "stop_def" || slot == "wall" || slot == "freekick_support" ||
      slot == "kickoff_def" || slot == "penalty_behind") {
    slot_w = normalizeAngle(std::atan2(ball_y - slot_y, ball_x - slot_x));
  } else if (slot == "stop_shadow") {
    slot_w = normalizeAngle(std::atan2(0.0 - slot_y, opp_goal_x - slot_x));
  } else {
    slot_w = normalizeAngle(slot_w);
  }

  setOutput("slot_x", slot_x);
  setOutput("slot_y", slot_y);
  setOutput("slot_w", slot_w);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace oxebots_strategy
