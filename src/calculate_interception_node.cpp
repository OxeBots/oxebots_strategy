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

  double dx_g = intercept_x - goal_x;
  double dy_g = intercept_y - goal_y;
  double dist_ball_goal = std::hypot(dx_g, dy_g);

  double pk_x = intercept_x;
  double pk_y = intercept_y;

  if (dist_ball_goal > 10.0) {
    double pre_kick_dist = 400.0; // mm
    pk_x = intercept_x + (dx_g / dist_ball_goal) * pre_kick_dist;
    pk_y = intercept_y + (dy_g / dist_ball_goal) * pre_kick_dist;
  }

  setOutput("intercept_pk_x", pk_x);
  setOutput("intercept_pk_y", pk_y);

  // --- Waypoint de contorno (intercept_pk_safe_x/y) ---
  // O D* só evita colidir com a bola, sem noção de "por qual lado contornar" pra chegar em pk_x/y
  // (atrás da bola) — se o robô já está do lado errado, ir direto arrisca um atalho pela frente da
  // bola. Quando isso acontece, mira num waypoint deslocado lateralmente (perpendicular à linha
  // bola-gol, do lado em que o robô já está) longe o bastante pra forçar o D* a rodear. Assim que
  // o robô está seguramente do lado certo, o alvo volta a ser pk_x/y diretamente.
  {
    // dx_g/dy_g aqui é (bola - gol), o oposto do "a=robô-bola, b=gol-bola" usado em
    // go_to_point_node.cpp — logo dot(a,b)>0 (lado errado) equivale a dot_side<0 aqui.
    double dot_side = dx_g * (rx - intercept_x) + dy_g * (ry - intercept_y);
    constexpr double kSafelyBehindMarginMm = 80.0; // histerese: só desarma com folga real
    if (dot_side < 0.0) {
      swing_active_ = true;
    } else if (dot_side > kSafelyBehindMarginMm) {
      swing_active_ = false;
    }
  }

  double pk_safe_x = pk_x;
  double pk_safe_y = pk_y;
  if (swing_active_ && dist_ball_goal > 10.0) {
    double back_x = dx_g / dist_ball_goal;
    double back_y = dy_g / dist_ball_goal;
    double perp_x = -back_y; // perpendicular a back_dir; o sinal segue o lado do robô
    double perp_y = back_x;
    double side_sign = (perp_x * (rx - intercept_x) + perp_y * (ry - intercept_y)) >= 0.0 ? 1.0 : -1.0;
    pk_safe_x = intercept_x + back_x * 400.0 + perp_x * side_sign * kSwingLateralOffsetMm;
    pk_safe_y = intercept_y + back_y * 400.0 + perp_y * side_sign * kSwingLateralOffsetMm;
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
  }
  setOutput("intercept_capture_x", cap_x);
  setOutput("intercept_capture_y", cap_y);

  // --- Ponto de Carga e Ponto de Mira da Carga: usados só pela árvore de carregar a bola.
  // Base é a posição BRUTA da bola (bx/by), não intercept_x/y (predição balística) como
  // pk/capture acima: ao carregar, o robô já está em contato empurrando, então a pergunta certa é
  // "onde a bola está AGORA", não uma previsão — usar a predição fazia o alvo deslizar pra frente
  // da bola de verdade durante o empurrão.
  double dx_g_raw = bx - goal_x;
  double dy_g_raw = by - goal_y;
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
    // Do lado do gol (sinal oposto).
    carry_face_x = bx - (dx_g_raw / dist_ball_goal_raw) * carry_face_lookahead_mm;
    carry_face_y = by - (dy_g_raw / dist_ball_goal_raw) * carry_face_lookahead_mm;
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
