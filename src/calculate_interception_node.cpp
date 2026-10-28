#include "oxebots_strategy/calculate_interception_node.h"
#include "oxebots_strategy/kick_readiness.hpp"
#include <cmath>

namespace oxebots_strategy
{

CalculateInterceptionNode::CalculateInterceptionNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  ball_pred_sub_ = node_->create_subscription<oxebots_interfaces::msg::BallPrediction>(
    "/ball_predicted", 10, std::bind(&CalculateInterceptionNode::ballPredictionCallback, this, std::placeholders::_1));

  robot_pred_sub_ = node_->create_subscription<oxebots_interfaces::msg::RobotPrediction>(
    "/robot_predicted", 10, std::bind(&CalculateInterceptionNode::robotPredictionCallback, this, std::placeholders::_1));

  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/interception_intent_marker", 10);

  RCLCPP_INFO(node_->get_logger(), "CalculateInterceptionNode configurado.");
}

BT::PortsList CalculateInterceptionNode::providedPorts()
{
  return { BT::InputPort<unsigned int>("robot_id"),
           BT::InputPort<double>("goal_x", 0.0, "X do gol alvo"),
           BT::InputPort<double>("goal_y", 0.0, "Y do gol alvo"),
           BT::InputPort<double>("carry_distance_mm", kCarryDistanceMm,
               "Distância (mm) do ponto de carga até a bola, do lado do gol. Ver kCarryDistanceMm."),
           BT::InputPort<double>("carry_face_lookahead_mm", kCarryFaceLookaheadMm,
               "Distância (mm) do ponto de mira da carga até a bola, do lado do gol."),
           BT::OutputPort<double>("intercept_x"),
           BT::OutputPort<double>("intercept_y"),
           BT::OutputPort<double>("intercept_pk_x"),
           BT::OutputPort<double>("intercept_pk_y"),
           BT::OutputPort<double>("intercept_pk_safe_x",
               "Igual a intercept_pk_x, exceto quando o robô está do lado errado da bola agora — "
               "nesse caso, um waypoint de contorno lateral em vez do pré-chute direto."),
           BT::OutputPort<double>("intercept_pk_safe_y"),
           BT::OutputPort<double>("intercept_capture_x"),
           BT::OutputPort<double>("intercept_capture_y"),
           BT::OutputPort<double>("intercept_carry_x"),
           BT::OutputPort<double>("intercept_carry_y"),
           BT::OutputPort<double>("intercept_carry_face_x"),
           BT::OutputPort<double>("intercept_carry_face_y"),
           BT::OutputPort<bool>("is_ready_to_kick") };
}

void CalculateInterceptionNode::ballPredictionCallback(const oxebots_interfaces::msg::BallPrediction::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_ball_pred_ = msg;
}

void CalculateInterceptionNode::robotPredictionCallback(const oxebots_interfaces::msg::RobotPrediction::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_robot_pred_ = msg;
}

BT::NodeStatus CalculateInterceptionNode::onStart()
{
  return onRunning();
}

BT::NodeStatus CalculateInterceptionNode::onRunning()
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (!last_ball_pred_ || !last_robot_pred_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "Nenhum dado predito (ball/robot) recebido, aguardando predições...");
    return BT::NodeStatus::RUNNING;
  }

  unsigned int robot_id;
  if (!getInput<unsigned int>("robot_id", robot_id)) {
    RCLCPP_ERROR(node_->get_logger(), "Falha ao ler o robot_id no CalculateInterceptionNode.");
    return BT::NodeStatus::FAILURE;
  }

  // Encontrar o nosso robô aliado predito
  bool found_robot = false;
  oxebots_interfaces::msg::RobotPredictionData my_robot;
  for (const auto& ally : last_robot_pred_->allies) {
    if (ally.id == robot_id) {
      my_robot = ally;
      found_robot = true;
      break;
    }
  }

  if (!found_robot) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "Robo %u nao encontrado nas predicoes de aliados!", robot_id);
    return BT::NodeStatus::RUNNING;
  }

  // Posição e velocidade atuais do robô (em mm e mm/s)
  double rx = my_robot.x;
  double ry = my_robot.y;
  double r_vx = my_robot.vx;
  double r_vy = my_robot.vy;

  // Posição, velocidade e aceleração iniciais da bola (em mm, mm/s, mm/s2)
  double bx = last_ball_pred_->x;
  double by = last_ball_pred_->y;
  double b_vx = last_ball_pred_->vx;
  double b_vy = last_ball_pred_->vy;
  double b_ax = last_ball_pred_->ax;
  double b_ay = last_ball_pred_->ay;

  // Banda-morta: com a bola parada, ruído de alguns cm/s na predição do Kalman é amplificado pela
  // busca abaixo (extrapola até 3s no futuro), fazendo o ponto de interceptação pular entre
  // lugares bem diferentes a cada tick. Abaixo do limiar, zera velocidade/aceleração preditas — o
  // ponto de interceptação vira a posição atual da bola, sem afetar a predição real quando a bola
  // está genuinamente em movimento rápido.
  constexpr double kBallStationarySpeedMmS = 150.0;
  if (std::hypot(b_vx, b_vy) < kBallStationarySpeedMmS) {
    b_vx = 0.0;
    b_vy = 0.0;
    b_ax = 0.0;
    b_ay = 0.0;
  }

  // Limitações físicas do robô
  double max_speed = 1500.0; // mm/s
  double max_accel = 2000.0; // mm/s^2

  // Tenta ler dos parâmetros do nó se disponíveis (padrão em metros/s e m/s^2)
  double param_speed = 1.5;
  if (node_->get_parameter("max_linear_speed", param_speed)) {
    max_speed = param_speed * 1000.0;
  }

  // Algoritmo de busca iterativa
  double intercept_x = bx;
  double intercept_y = by;
  bool solved = false;

  double b_speed = std::hypot(b_vx, b_vy);
  double b_accel = std::hypot(b_ax, b_ay);

  // Calcular tempo de parada da bola (se estiver desacelerando)
  double t_ball_stop = (b_accel > 1.0) ? (b_speed / b_accel) : 0.0;

  for (double t = 0.05; t <= 3.0; t += 0.05) {
    // Onde a bola estará no instante t?
    double x_t, y_t;
    if (t_ball_stop > 0.0 && t >= t_ball_stop) {
      // A bola já parou neste instante
      x_t = bx + b_vx * t_ball_stop + 0.5 * b_ax * t_ball_stop * t_ball_stop;
      y_t = by + b_vy * t_ball_stop + 0.5 * b_ay * t_ball_stop * t_ball_stop;
    } else {
      x_t = bx + b_vx * t + 0.5 * b_ax * t * t;
      y_t = by + b_vy * t + 0.5 * b_ay * t * t;
    }

    // Distância entre o robô e a bola na posição futura
    double dx = x_t - rx;
    double dy = y_t - ry;
    double dist = std::hypot(dx, dy);

    if (dist < 1e-3) {
      intercept_x = x_t;
      intercept_y = y_t;
      solved = true;
      break;
    }

    // Direção para o alvo futuro
    double ux = dx / dist;
    double uy = dy / dist;

    // Projetar velocidade do robô na direção do alvo
    double v0 = r_vx * ux + r_vy * uy;

    double t_travel = 0.0;
    if (v0 < 0.0) {
      // O robô está indo na direção contrária, precisa frear antes de acelerar
      double t_brake = -v0 / max_accel;
      double d_brake = (v0 * v0) / (2.0 * max_accel);
      double d_remaining = dist + d_brake;

      double d_acc = (max_speed * max_speed) / (2.0 * max_accel);

      if (d_remaining < 2.0 * d_acc) {
        t_travel = t_brake + 2.0 * std::sqrt(d_remaining / max_accel);
      } else {
        t_travel = t_brake + (max_speed / max_accel) + (d_remaining / max_speed);
      }
    } else {
      // O robô está se movendo na direção certa ou parado
      double d_acc = (max_speed * max_speed - v0 * v0) / (2.0 * max_accel);
      if (d_acc < 0.0) d_acc = 0.0;
      double d_dec = (max_speed * max_speed) / (2.0 * max_accel);

      if (dist < (d_acc + d_dec)) {
        double v_peak = std::sqrt((2.0 * max_accel * dist + v0 * v0) / 2.0);
        if (v_peak > max_speed) v_peak = max_speed;
        t_travel = (2.0 * v_peak - v0) / max_accel;
      } else {
        double t_acc = (max_speed - v0) / max_accel;
        double t_dec = max_speed / max_accel;
        double t_cruise = (dist - (d_acc + d_dec)) / max_speed;
        t_travel = t_acc + t_dec + t_cruise;
      }
    }

    if (t_travel <= t) {
      intercept_x = x_t;
      intercept_y = y_t;
      solved = true;
      break;
    }
  }

  if (!solved) {
    // Se não encontrou interseção, vai para o ponto final onde a bola para
    if (t_ball_stop > 0.0) {
      intercept_x = bx + b_vx * t_ball_stop + 0.5 * b_ax * t_ball_stop * t_ball_stop;
      intercept_y = by + b_vy * t_ball_stop + 0.5 * b_ay * t_ball_stop * t_ball_stop;
    } else {
      intercept_x = bx + b_vx * 3.0 + 0.5 * b_ax * 9.0;
      intercept_y = by + b_vy * 3.0 + 0.5 * b_ay * 9.0;
    }
  }

  setOutput("intercept_x", intercept_x);
  setOutput("intercept_y", intercept_y);

  // --- Calcular Ponto Atrás da Bola Futura (Pre-Kick Futuro) ---
  double goal_x = 0.0, goal_y = 0.0;
  getInput<double>("goal_x", goal_x);
  getInput<double>("goal_y", goal_y);

  // Ler geometria do campo e da nossa área de pênalti do Blackboard para evitar posicionar
  // pk / pk_safe / capture / carry dentro da própria área (e para contornar pela lateral/frente).
  double my_goal_x = -goal_x;
  double p_depth = 0.0;
  double p_width = 0.0;
  double field_length = 0.0;
  double field_width = 0.0;
  auto root_bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
  if (root_bb) {
    (void)root_bb->get("my_goal_x", my_goal_x);
    (void)root_bb->get("penalty_area_depth", p_depth);
    (void)root_bb->get("penalty_area_width", p_width);
    (void)root_bb->get("field_length", field_length);
    (void)root_bb->get("field_width", field_width);
  }

  // Área aliada sem margem extra (0mm); área inimiga com margem de 8cm (80mm), aplicável aos dois lados.
  constexpr double kAllyAreaMarginMm = 0.0;
  constexpr double kEnemyAreaMarginMm = 80.0;
  const bool restrict_our_area = (robot_id != 0 && p_depth > 0.0 && p_width > 0.0 && std::abs(my_goal_x) > 1.0);
  const bool restrict_opp_area = (p_depth > 0.0 && p_width > 0.0 && std::abs(goal_x) > 1.0);
  const double area_front_x = (my_goal_x < 0.0)
      ? (my_goal_x + p_depth + kAllyAreaMarginMm)
      : (my_goal_x - p_depth - kAllyAreaMarginMm);
  const double area_side_y = (p_width / 2.0) + kAllyAreaMarginMm;

  // Limita a bola predita dentro das dimensões do campo para evitar extrapolação para fora nas bordas/laterais
  if (field_length > 0.0 && field_width > 0.0) {
    intercept_x = std::clamp(intercept_x, -field_length / 2.0, field_length / 2.0);
    intercept_y = std::clamp(intercept_y, -field_width / 2.0, field_width / 2.0);
    setOutput("intercept_x", intercept_x);
    setOutput("intercept_y", intercept_y);
  }

  const double opp_area_front_x = (goal_x < 0.0)
      ? (goal_x + p_depth + kEnemyAreaMarginMm)
      : (goal_x - p_depth - kEnemyAreaMarginMm);
  const double opp_area_side_y = (p_width / 2.0) + kEnemyAreaMarginMm;

  auto is_inside_our_area = [&](double x, double y) -> bool {
    if (!restrict_our_area) return false;
    bool in_x = (my_goal_x < 0.0) ? (x < area_front_x) : (x > area_front_x);
    return in_x && (std::abs(y) < area_side_y);
  };

  auto is_inside_opp_area = [&](double x, double y) -> bool {
    if (!restrict_opp_area) return false;
    bool in_x = (goal_x < 0.0) ? (x < opp_area_front_x) : (x > opp_area_front_x);
    return in_x && (std::abs(y) < opp_area_side_y);
  };

  auto clamp_out_our_area = [&](double& x, double& y, double ref_bx, double ref_by) {
    if (!is_inside_our_area(x, y)) return;
    double pen_x = (my_goal_x < 0.0) ? (area_front_x - x) : (x - area_front_x);
    double pen_y = area_side_y - std::abs(y);
    bool ball_on_side = ((my_goal_x < 0.0) ? (ref_bx < area_front_x) : (ref_bx > area_front_x)) &&
                        (std::abs(ref_by) >= (p_width / 2.0) * 0.9);
    if (pen_y < pen_x || ball_on_side) {
      double sy = (std::abs(ref_by) > 1.0) ? ((ref_by >= 0.0) ? 1.0 : -1.0) : ((y >= 0.0) ? 1.0 : -1.0);
      y = sy * area_side_y;
    } else {
      x = area_front_x;
    }
  };

  auto clamp_out_opp_area = [&](double& x, double& y, double ref_bx, double ref_by) {
    if (!is_inside_opp_area(x, y)) return;
    double pen_x = (goal_x < 0.0) ? (opp_area_front_x - x) : (x - opp_area_front_x);
    double pen_y = opp_area_side_y - std::abs(y);
    bool ball_on_side = ((goal_x < 0.0) ? (ref_bx < opp_area_front_x) : (ref_bx > opp_area_front_x)) &&
                        (std::abs(ref_by) >= (p_width / 2.0) * 0.9);
    if (pen_y < pen_x || ball_on_side) {
      double sy = (std::abs(ref_by) > 1.0) ? ((ref_by >= 0.0) ? 1.0 : -1.0) : ((y >= 0.0) ? 1.0 : -1.0);
      y = sy * opp_area_side_y;
    } else {
      x = opp_area_front_x;
    }
  };

  double dx_g = intercept_x - goal_x;
  double dy_g = intercept_y - goal_y;
  double dist_ball_goal = std::hypot(dx_g, dy_g);

  double pk_x = intercept_x;
  double pk_y = intercept_y;
  constexpr double kPreKickDistMm = 400.0;
  bool area_adjusted = false;
  bool ball_on_lateral = restrict_our_area &&
      ((my_goal_x < 0.0) ? (intercept_x < area_front_x + 200.0)
                         : (intercept_x > area_front_x - 200.0)) &&
      (std::abs(intercept_y) >= (p_width / 2.0) - 50.0);

  if (dist_ball_goal > 10.0) {
    pk_x = intercept_x + (dx_g / dist_ball_goal) * kPreKickDistMm;
    pk_y = intercept_y + (dy_g / dist_ball_goal) * kPreKickDistMm;

    // 1. Se a bola está na lateral da área, impõe o limite de coordenada Y baseado na linha lateral
    // da área (|pk_y| >= area_side_y), mantendo pk_x atrás da bola (em direção ao fundo do campo)
    // e alinhando a condução/chute paralelamente à linha lateral da área.
    if (ball_on_lateral) {
      area_adjusted = true;
      double sy = (intercept_y >= 0.0) ? 1.0 : -1.0;
      pk_y = sy * std::max(std::abs(intercept_y), area_side_y);
      double dy_pk = std::min(std::abs(pk_y - intercept_y), kPreKickDistMm * 0.8);
      double rem_x = std::sqrt(std::max(0.0, kPreKickDistMm * kPreKickDistMm - dy_pk * dy_pk));
      pk_x = intercept_x + std::copysign(rem_x, my_goal_x);
      // Alinha paralelo à lateral da área (em direção ao campo adversário) para não cortar pela quina
      dx_g = std::copysign(dist_ball_goal, my_goal_x);
      dy_g = 0.0;
    } else if (is_inside_our_area(pk_x, pk_y)) {
      // 2. Bola na frente da área: impõe o limite de coordenada X na linha frontal da área (area_front_x)
      // e desloca pk_y lateralmente para posicionar o robô ao lado/atrás da bola sem entrar na área.
      area_adjusted = true;
      double avail_dx = (my_goal_x < 0.0)
          ? std::clamp(intercept_x - area_front_x, 0.0, kPreKickDistMm)
          : std::clamp(area_front_x - intercept_x, 0.0, kPreKickDistMm);
      double rem_y = std::sqrt(std::max(0.0, kPreKickDistMm * kPreKickDistMm - avail_dx * avail_dx));
      if (intercept_y > 150.0) {
        last_clear_sign_ = 1.0;
      } else if (intercept_y < -150.0) {
        last_clear_sign_ = -1.0;
      } else if (!swing_active_ && std::hypot(rx - intercept_x, ry - intercept_y) > 450.0) {
        last_clear_sign_ = (ry <= intercept_y) ? 1.0 : -1.0;
      }
      double clear_sign = last_clear_sign_;
      pk_x = (my_goal_x < 0.0)
          ? std::max(area_front_x, intercept_x - avail_dx)
          : std::min(area_front_x, intercept_x + avail_dx);
      pk_y = intercept_y - clear_sign * rem_y;

      // Direção efetiva de chute/carga colinear com o vetor real (pk -> bola), garantindo que a reta
      // de avanço (pk -> carry) passe exatamente pelo centro da bola sem errar pela frente!
      double adj_dx = std::copysign(std::max(avail_dx, 15.0), my_goal_x);
      double adj_dy = -clear_sign * rem_y;
      double adj_norm = std::hypot(adj_dx, adj_dy);
      if (adj_norm > 1e-3) {
        dx_g = (adj_dx / adj_norm) * dist_ball_goal;
        dy_g = (adj_dy / adj_norm) * dist_ball_goal;
      }
    }
  }

  clamp_out_opp_area(pk_x, pk_y, intercept_x, intercept_y);
  setOutput("intercept_pk_x", pk_x);
  setOutput("intercept_pk_y", pk_y);

  // --- Waypoint de contorno (intercept_pk_safe_x/y) ---
  // O D* só evita colidir com a bola, sem noção de "por qual lado contornar" pra chegar em pk_x/y
  // (atrás da bola) — se o robô já está do lado errado, ir direto arrisca um atalho pela frente da
  // bola. Quando isso acontece, mira num waypoint deslocado lateralmente (perpendicular à linha
  // bola-gol, do lado em que o robô já está) longe o bastante pra forçar o D* a rodear. Assim que
  // o robô está seguramente do lado certo, o alvo volta a ser pk_x/y diretamente.
  {
    // Projeção normalizada (em mm) do vetor (robô - bola) na direção "atrás da bola" (dx_g, dy_g):
    // dot_side < 0 significa que o robô está na frente da bola; > 80mm significa que já passou para trás.
    double ux_g = (dist_ball_goal > 10.0) ? (dx_g / dist_ball_goal) : 0.0;
    double uy_g = (dist_ball_goal > 10.0) ? (dy_g / dist_ball_goal) : 0.0;
    double dot_side = ux_g * (rx - intercept_x) + uy_g * (ry - intercept_y);
    constexpr double kSafelyBehindMarginMm = 80.0; // histerese em mm reais: só desarma com folga real
    if (dot_side < 0.0) {
      swing_active_ = true;
    } else if (dot_side > kSafelyBehindMarginMm) {
      swing_active_ = false;
    }
  }

  double pk_safe_x = pk_x;
  double pk_safe_y = pk_y;
  if (swing_active_ && dist_ball_goal > 10.0) {
    if (ball_on_lateral) {
      // Na lateral da área: mantém o mesmo pk_x e apenas garante uma folga lateral suave por fora da bola
      // (em direção à lateral do campo), evitando saltos bruscos de coordenada ao desarmar swing_active_.
      double sy = (intercept_y >= 0.0) ? 1.0 : -1.0;
      pk_safe_x = pk_x;
      pk_safe_y = sy * std::max(std::abs(intercept_y) + 180.0, area_side_y);
    } else {
      double back_x = dx_g / dist_ball_goal;
      double back_y = dy_g / dist_ball_goal;
      double perp_x = -back_y; // perpendicular a back_dir; o sinal segue o lado do robô com histerese
      double perp_y = back_x;
      double perp_proj = perp_x * (rx - intercept_x) + perp_y * (ry - intercept_y);
      if (perp_proj > 120.0) {
        last_side_sign_ = 1.0;
      } else if (perp_proj < -120.0) {
        last_side_sign_ = -1.0;
      }
      double side_sign = last_side_sign_;
      pk_safe_x = intercept_x + back_x * 400.0 + perp_x * side_sign * kSwingLateralOffsetMm;
      pk_safe_y = intercept_y + back_y * 400.0 + perp_y * side_sign * kSwingLateralOffsetMm;

      // Se o waypoint cair dentro da área aliada ou adversária, inverte o lado para contornar pelo campo aberto
      if (is_inside_our_area(pk_safe_x, pk_safe_y) || is_inside_opp_area(pk_safe_x, pk_safe_y)) {
        side_sign = -side_sign;
        last_side_sign_ = side_sign;
        pk_safe_x = intercept_x + back_x * 400.0 + perp_x * side_sign * kSwingLateralOffsetMm;
        pk_safe_y = intercept_y + back_y * 400.0 + perp_y * side_sign * kSwingLateralOffsetMm;
      }
    }

    // Mantém pk_safe dentro dos limites do campo e fora da área
    if (field_length > 0.0 && field_width > 0.0) {
      double max_fx = (field_length / 2.0) - 100.0;
      double max_fy = (field_width / 2.0) - 100.0;
      pk_safe_x = std::clamp(pk_safe_x, -max_fx, max_fx);
      pk_safe_y = std::clamp(pk_safe_y, -max_fy, max_fy);
    }
    clamp_out_our_area(pk_safe_x, pk_safe_y, intercept_x, intercept_y);
    clamp_out_opp_area(pk_safe_x, pk_safe_y, intercept_x, intercept_y);
  }
  setOutput("intercept_pk_safe_x", pk_safe_x);
  setOutput("intercept_pk_safe_y", pk_safe_y);

  // --- Ponto de Captura: mesma linha gol->bola, mas bem perto da bola ---
  // Usado como alvo do GoToPoint em vez do centro exato da bola (intercept_x/y), para que o
  // robô termine a aproximação já alinhado, em vez de bater na bola de frente.
  double cap_x = intercept_x;
  double cap_y = intercept_y;
  if (dist_ball_goal > 10.0) {
    cap_x = intercept_x + (dx_g / dist_ball_goal) * kCaptureDistanceMm;
    cap_y = intercept_y + (dy_g / dist_ball_goal) * kCaptureDistanceMm;
    clamp_out_our_area(cap_x, cap_y, intercept_x, intercept_y);
    clamp_out_opp_area(cap_x, cap_y, intercept_x, intercept_y);
  }
  setOutput("intercept_capture_x", cap_x);
  setOutput("intercept_capture_y", cap_y);

  // --- Ponto de Carga e Ponto de Mira da Carga: usados só pela árvore de carregar a bola.
  // Base é a posição BRUTA da bola (bx/by), não intercept_x/y (predição balística) como
  // pk/capture acima: ao carregar, o robô já está em contato empurrando, então a pergunta certa é
  // "onde a bola está AGORA", não uma previsão — usar a predição fazia o alvo deslizar pra frente
  // da bola de verdade durante o empurrão.
  // Quando perto da área aliada (area_adjusted), reutiliza a direção ajustada (dx_g, dy_g) para não
  // empurrar o robô para dentro da área durante o Carry.
  double dx_g_raw = (area_adjusted && dist_ball_goal > 10.0) ? dx_g : (bx - goal_x);
  double dy_g_raw = (area_adjusted && dist_ball_goal > 10.0) ? dy_g : (by - goal_y);
  double dist_ball_goal_raw = std::hypot(dx_g_raw, dy_g_raw);

  double carry_distance_mm = kCarryDistanceMm;
  getInput<double>("carry_distance_mm", carry_distance_mm);
  double carry_face_lookahead_mm = kCarryFaceLookaheadMm;
  getInput<double>("carry_face_lookahead_mm", carry_face_lookahead_mm);

  double carry_x = bx;
  double carry_y = by;
  double carry_face_x = bx;
  double carry_face_y = by;
  if (dist_ball_goal_raw > 10.0) {
    // Do lado OPOSTO ao gol (mesmo sinal que pk/capture usam para dx_g/dy_g).
    carry_x = bx + (dx_g_raw / dist_ball_goal_raw) * carry_distance_mm;
    carry_y = by + (dy_g_raw / dist_ball_goal_raw) * carry_distance_mm;
    clamp_out_our_area(carry_x, carry_y, bx, by);
    clamp_out_opp_area(carry_x, carry_y, bx, by);
    // Do lado do gol (sinal oposto). Usa distância mínima de 2500mm para que GoToPointNode::resolveFaceAngle
    // (atan2(face - target_pos_)) mantenha o ângulo da reta bola->alvo sem distorção angular quando
    // target_pos_ alterna entre pk_safe (deslocado 600mm) e pk.
    double effective_face_lookahead = std::max(carry_face_lookahead_mm, 2500.0);
    carry_face_x = bx - (dx_g_raw / dist_ball_goal_raw) * effective_face_lookahead;
    carry_face_y = by - (dy_g_raw / dist_ball_goal_raw) * effective_face_lookahead;
  }
  setOutput("intercept_carry_x", carry_x);
  setOutput("intercept_carry_y", carry_y);
  setOutput("intercept_carry_face_x", carry_face_x);
  setOutput("intercept_carry_face_y", carry_face_y);

  // --- Checar se o robô já está no pré-chute pronto para o PrecisionKick ---
  double r_dx = intercept_x - rx;
  double r_dy = intercept_y - ry;
  double dist_to_pk = std::hypot(pk_x - rx, pk_y - ry);

  double dot = (dx_g * r_dx + dy_g * r_dy);

  bool current_ready = false;
  (void)config().blackboard->get("is_ready_to_kick", current_ready);

  bool is_ready = computeKickReadiness(current_ready, dist_to_pk, dot);

  setOutput("is_ready_to_kick", is_ready);

  // Publicar marcadores visuais no RViz
  publishMarkers(intercept_x, intercept_y, pk_x, pk_y, rx, ry, is_ready);

  return BT::NodeStatus::SUCCESS;
}

void CalculateInterceptionNode::onHalted()
{
}

void CalculateInterceptionNode::publishMarkers(double ball_x, double ball_y, double pk_x, double pk_y, double rx, double ry, bool is_ready)
{
  (void)rx; (void)ry;

  // Marcador 1: Ponto futuro estimado da bola (Esfera vermelha semi-transparente)
  auto ball_marker = visualization_msgs::msg::Marker();
  ball_marker.header.frame_id = "map";
  ball_marker.header.stamp = node_->now();
  ball_marker.ns = "intercept_ball";
  ball_marker.id = 0;
  ball_marker.type = visualization_msgs::msg::Marker::SPHERE;
  ball_marker.action = visualization_msgs::msg::Marker::ADD;
  ball_marker.pose.position.x = ball_x / 1000.0;
  ball_marker.pose.position.y = ball_y / 1000.0;
  ball_marker.pose.position.z = 0.05;
  ball_marker.scale.x = 0.15; ball_marker.scale.y = 0.15; ball_marker.scale.z = 0.15;
  ball_marker.color.a = 0.6;
  ball_marker.color.r = 1.0; ball_marker.color.g = 0.0; ball_marker.color.b = 0.0;
  marker_pub_->publish(ball_marker);

  // Marcador 2: Ponto de pré-chute futuro (Cilindro azul/verde semi-transparente)
  auto pk_marker = visualization_msgs::msg::Marker();
  pk_marker.header.frame_id = "map";
  pk_marker.header.stamp = node_->now();
  pk_marker.ns = "intercept_pk";
  pk_marker.id = 1;
  pk_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  pk_marker.action = visualization_msgs::msg::Marker::ADD;
  pk_marker.pose.position.x = pk_x / 1000.0;
  pk_marker.pose.position.y = pk_y / 1000.0;
  pk_marker.pose.position.z = 0.02;
  pk_marker.scale.x = 0.2; pk_marker.scale.y = 0.2; pk_marker.scale.z = 0.04;
  pk_marker.color.a = 0.7;
  if (is_ready) {
    pk_marker.color.r = 0.0; pk_marker.color.g = 1.0; pk_marker.color.b = 0.0; // Verde
  } else {
    pk_marker.color.r = 0.0; pk_marker.color.g = 0.5; pk_marker.color.b = 1.0; // Azul/Ciano
  }
  marker_pub_->publish(pk_marker);
}

} // namespace oxebots_strategy
