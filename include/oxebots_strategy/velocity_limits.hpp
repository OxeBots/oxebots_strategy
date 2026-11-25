// Utilitários puros para limites mínimo e máximo de velocidade (linear e angular)
// e estimativa cinemática de tempo de deslocamento adaptada ao intervalo [min, max].
//
// Sem dependências de ROS ou BehaviorTree.CPP para permitir reutilização uniforme
// entre movement_calculation_node, folhas da Behavior Tree e testes unitários isolados.
#ifndef OXEBOTS_STRATEGY__VELOCITY_LIMITS_HPP_
#define OXEBOTS_STRATEGY__VELOCITY_LIMITS_HPP_

#include <algorithm>
#include <cmath>

namespace movement
{

inline double sanitizeSpeedLimit(double value)
{
  if (!std::isfinite(value) || value < 0.0) {
    return 0.0;
  }
  return value;
}

// Piso efetivo de velocidade: nunca pode ultrapassar o teto máximo vigente.
// Quando o teto é <= 0 (ex.: HALT, TIMEOUT, watchdog), o piso efetivo é 0.
inline double effectiveMinSpeed(double min_speed, double max_speed)
{
  const double max_s = sanitizeSpeedLimit(max_speed);
  if (max_s <= 0.0) {
    return 0.0;
  }
  const double min_s = sanitizeSpeedLimit(min_speed);
  return std::min(min_s, max_s);
}

// Aplica piso mínimo e teto máximo sobre uma magnitude de velocidade linear (m/s).
// Comandos nulos ou abaixo de zero_eps permanecem exatamente 0.0 (respeita parada,
// HALT, chegada ao alvo e rotação no lugar).
inline double applyMinMaxLinearSpeed(
  double speed, double min_speed, double max_speed, double zero_eps = 1e-4)
{
  if (!std::isfinite(speed) || speed <= zero_eps) {
    return 0.0;
  }
  const double max_s = sanitizeSpeedLimit(max_speed);
  if (max_s <= 0.0) {
    return 0.0;
  }
  const double min_s = effectiveMinSpeed(min_speed, max_s);
  return std::clamp(speed, min_s, max_s);
}

// Aplica piso mínimo e teto máximo preservando a direção do vetor (vx, vy).
// Vetores nulos (módulo <= zero_eps) permanecem (0, 0).
inline void applyMinMaxLinearVector(
  float & vx, float & vy, double min_speed, double max_speed, double zero_eps = 1e-4)
{
  if (!std::isfinite(vx)) {
    vx = 0.0f;
  }
  if (!std::isfinite(vy)) {
    vy = 0.0f;
  }
  const double speed = std::hypot(static_cast<double>(vx), static_cast<double>(vy));
  if (speed <= zero_eps) {
    vx = 0.0f;
    vy = 0.0f;
    return;
  }
  const double clamped = applyMinMaxLinearSpeed(speed, min_speed, max_speed, zero_eps);
  if (clamped <= 0.0) {
    vx = 0.0f;
    vy = 0.0f;
    return;
  }
  const double scale = clamped / speed;
  vx = static_cast<float>(static_cast<double>(vx) * scale);
  vy = static_cast<float>(static_cast<double>(vy) * scale);
}

// Aplica piso mínimo e teto máximo preservando o sinal da velocidade angular (rad/s).
// Comandos nulos (|omega| <= zero_eps) permanecem 0.0.
inline double applyMinMaxAngularSpeed(
  double omega, double min_angular, double max_angular, double zero_eps = 1e-4)
{
  if (!std::isfinite(omega) || std::abs(omega) <= zero_eps) {
    return 0.0;
  }
  const double max_a = sanitizeSpeedLimit(max_angular);
  if (max_a <= 0.0) {
    return 0.0;
  }
  const double min_a = effectiveMinSpeed(min_angular, max_a);
  const double mag = std::clamp(std::abs(omega), min_a, max_a);
  return std::copysign(mag, omega);
}

// Zona morta linear adaptativa na chegada ao alvo: evita oscilação (hunting) quando
// min_linear_speed é alto em relação ao passo de 1 ciclo de controle, sem ultrapassar
// as tolerâncias mínimas usadas pelas Behavior Trees.
inline double effectiveArrivalDistanceMm(
  double base_arrived_mm, double min_linear_mps, double max_linear_mps,
  double control_dt_s = 0.02, double max_arrived_mm = 25.0)
{
  const double base = sanitizeSpeedLimit(base_arrived_mm);
  const double eff_min = effectiveMinSpeed(min_linear_mps, max_linear_mps);
  if (eff_min <= 0.0) {
    return base;
  }
  const double step_mm = eff_min * 1000.0 * sanitizeSpeedLimit(control_dt_s);
  return std::clamp(std::max(base, step_mm), base, std::max(base, max_arrived_mm));
}

// Zona morta angular adaptativa: evita oscilação de yaw quando min_angular_speed é alto
// em relação ao passo de 1 ciclo de controle, permanecendo abaixo da tolerância das BTs.
inline double effectiveAngleToleranceRad(
  double base_tol_rad, double min_angular_rad_s, double max_angular_rad_s,
  double control_dt_s = 0.02, double max_tol_rad = 0.12)
{
  const double base = sanitizeSpeedLimit(base_tol_rad);
  const double eff_min = effectiveMinSpeed(min_angular_rad_s, max_angular_rad_s);
  if (eff_min <= 0.0) {
    return base;
  }
  const double half_step_rad = 0.5 * eff_min * sanitizeSpeedLimit(control_dt_s);
  return std::clamp(std::max(base, half_step_rad), base, std::max(base, max_tol_rad));
}

// Estimativa do tempo (s) para percorrer dist_mm com aceleração max_accel_mm_s2,
// velocidade inicial projetada v0_mm_s e limites [min_speed_mm_s, max_speed_mm_s].
//
// Quando min_speed_mm_s == 0, equivale exatamente ao perfil trapezoidal/triangular clássico
// com parada no destino. Quando min_speed_mm_s > 0, considera que o controlador não
// desacelera abaixo de effective_min antes de alcançar a zona de chegada.
// Nunca retorna NaN/Inf para distâncias válidas e teto positivo.
inline double estimateTravelTimeS(
  double dist_mm, double v0_mm_s, double min_speed_mm_s,
  double max_speed_mm_s, double max_accel_mm_s2)
{
  if (!std::isfinite(dist_mm) || dist_mm <= 0.0) {
    return 0.0;
  }
  const double v_max = sanitizeSpeedLimit(max_speed_mm_s);
  const double accel = sanitizeSpeedLimit(max_accel_mm_s2);
  if (v_max <= 1e-6 || accel <= 1e-6) {
    return 0.0;
  }

  const double v_min = effectiveMinSpeed(min_speed_mm_s, v_max);
  const double v0 = std::isfinite(v0_mm_s) ? v0_mm_s : 0.0;

  double t_brake = 0.0;
  double d_rem = dist_mm;
  double v_start = v0;

  if (v0 < 0.0) {
    t_brake = -v0 / accel;
    const double d_brake = (v0 * v0) / (2.0 * accel);
    d_rem = dist_mm + d_brake;
    v_start = 0.0;
  } else if (v0 > v_max) {
    v_start = v_max;
  }

  const double v_end = v_min;

  // Caso 1: v_start > v_end e mesmo desacelerando continuamente até v_end excederia d_rem
  if (v_start > v_end) {
    const double d_min_dec = (v_start * v_start - v_end * v_end) / (2.0 * accel);
    if (d_rem <= d_min_dec) {
      const double disc = std::max(0.0, v_start * v_start - 2.0 * accel * d_rem);
      const double v_final = std::sqrt(disc);
      return t_brake + (v_start - v_final) / accel;
    }
  }

  // Caso 2: v_start < v_end e mesmo acelerando continuamente até v_end excederia d_rem
  if (v_start < v_end) {
    const double d_min_acc = (v_end * v_end - v_start * v_start) / (2.0 * accel);
    if (d_rem <= d_min_acc) {
      const double v_final = std::sqrt(v_start * v_start + 2.0 * accel * d_rem);
      return t_brake + (v_final - v_start) / accel;
    }
  }

  // Caso 3: perfil triangular ou trapezoidal atingindo v_peak em [max(v_start, v_end), v_max]
  const double d_acc_full = std::max(0.0, (v_max * v_max - v_start * v_start) / (2.0 * accel));
  const double d_dec_full = std::max(0.0, (v_max * v_max - v_end * v_end) / (2.0 * accel));

  if (d_rem < (d_acc_full + d_dec_full)) {
    const double v_peak_raw = std::sqrt(
      std::max(0.0, (2.0 * accel * d_rem + v_start * v_start + v_end * v_end) / 2.0));
    const double v_peak = std::clamp(v_peak_raw, std::max(v_start, v_end), v_max);
    const double t_move = ((v_peak - v_start) + (v_peak - v_end)) / accel;
    return t_brake + t_move;
  }

  const double t_acc = (v_max - v_start) / accel;
  const double t_dec = (v_max - v_end) / accel;
  const double t_cruise = (d_rem - (d_acc_full + d_dec_full)) / v_max;
  return t_brake + t_acc + t_dec + t_cruise;
}

// Projeta uma pose filtrada (mm, mm, rad) vinda do oxebots_prediction (/robot_predicted)
// para frente por (dt_since_s + latency_comp_s), com clamp seguro e wrap de ângulo em [-pi, pi].
inline void projectPredictedPoseMm(
  double & x_mm,
  double & y_mm,
  double & theta_rad,
  double vx_mps,
  double vy_mps,
  double omega_radps,
  double dt_since_s,
  double latency_comp_s,
  double max_horizon_s = 0.20)
{
  const double clamped_max = std::max(0.0, std::isfinite(max_horizon_s) ? max_horizon_s : 0.20);
  const double dt_since = std::clamp(std::isfinite(dt_since_s) ? dt_since_s : 0.0, 0.0, clamped_max);
  const double dt_lat = std::clamp(std::isfinite(latency_comp_s) ? latency_comp_s : 0.0, 0.0, clamped_max);
  const double total_dt = std::clamp(dt_since + dt_lat, 0.0, clamped_max);

  if (std::isfinite(vx_mps)) {
    x_mm += (vx_mps * 1000.0) * total_dt;
  }
  if (std::isfinite(vy_mps)) {
    y_mm += (vy_mps * 1000.0) * total_dt;
  }
  if (std::isfinite(omega_radps)) {
    theta_rad += omega_radps * total_dt;
    while (theta_rad > M_PI) theta_rad -= 2.0 * M_PI;
    while (theta_rad < -M_PI) theta_rad += 2.0 * M_PI;
  }
}

// Constrains the commanded velocity so the robot cannot approach another robot faster
// than the remaining separation allows. Positions and velocities are in meters and m/s.
inline bool enforceRobotSeparation(
  double self_x_m,
  double self_y_m,
  double other_x_m,
  double other_y_m,
  double other_vx_mps,
  double other_vy_mps,
  double min_separation_m,
  double response_rate,
  double fallback_away_x,
  double fallback_away_y,
  double & command_vx_mps,
  double & command_vy_mps)
{
  if (!std::isfinite(self_x_m) || !std::isfinite(self_y_m) ||
      !std::isfinite(other_x_m) || !std::isfinite(other_y_m) ||
      !std::isfinite(other_vx_mps) || !std::isfinite(other_vy_mps) ||
      !std::isfinite(command_vx_mps) || !std::isfinite(command_vy_mps) ||
      !std::isfinite(min_separation_m) || min_separation_m <= 0.0 ||
      !std::isfinite(response_rate) || response_rate <= 0.0) {
    return false;
  }

  double away_x = self_x_m - other_x_m;
  double away_y = self_y_m - other_y_m;
  const double distance = std::hypot(away_x, away_y);
  if (distance > 2.0 * min_separation_m) {
    return false;
  }

  if (distance > 1e-6) {
    away_x /= distance;
    away_y /= distance;
  } else {
    const double fallback_norm = std::hypot(fallback_away_x, fallback_away_y);
    if (!std::isfinite(fallback_norm) || fallback_norm <= 1e-6) {
      away_x = 1.0;
      away_y = 0.0;
    } else {
      away_x = fallback_away_x / fallback_norm;
      away_y = fallback_away_y / fallback_norm;
    }
  }

  const double other_radial_velocity = other_vx_mps * away_x + other_vy_mps * away_y;
  const double minimum_radial_velocity =
    other_radial_velocity - response_rate * (distance - min_separation_m);
  const double commanded_radial_velocity =
    command_vx_mps * away_x + command_vy_mps * away_y;
  if (commanded_radial_velocity >= minimum_radial_velocity) {
    return false;
  }

  const double correction = minimum_radial_velocity - commanded_radial_velocity;
  command_vx_mps += correction * away_x;
  command_vy_mps += correction * away_y;
  return true;
}

// Controle angular PD seguro com piso mínimo (min_angular_speed):
// Evita que um pequeno termo derivativo de frenagem (sinal oposto a angle_err e módulo
// menor que o piso mínimo) seja amplificado pelo piso em um giro reverso brusco (-min_angular),
// que causava chattering angular e desvios laterais em espiral.
inline double applyPdAngularSpeedWithFloor(
  double angle_err_rad,
  double omega_radps,
  double p_gain,
  double d_gain,
  double min_angular,
  double max_angular,
  double zero_eps = 1e-4)
{
  if (!std::isfinite(angle_err_rad) || !std::isfinite(p_gain)) {
    return 0.0;
  }
  const double d_eff = (std::isfinite(d_gain) && d_gain > 0.0) ? d_gain : 0.0;
  const double w_eff = std::isfinite(omega_radps) ? omega_radps : 0.0;
  const double pd_cmd = (angle_err_rad * p_gain) - (w_eff * d_eff);
  const double eff_min = effectiveMinSpeed(min_angular, max_angular);

  // Se o D-term inverteu o sinal em relação ao erro (frenagem leve) mas não atingiu o piso,
  // faz coast (0.0) em vez de amplificar a frenagem até -eff_min.
  if (eff_min > 0.0 && (angle_err_rad * pd_cmd) < 0.0 && std::abs(pd_cmd) < eff_min) {
    return 0.0;
  }
  return applyMinMaxAngularSpeed(pd_cmd, min_angular, max_angular, zero_eps);
}

// Controle linear PD 2D seguro com piso mínimo (min_linear_speed):
// - Quando damp_along_error == false (seguindo ponto de lookahead móvel em Pure Pursuit),
//   amortece apenas a velocidade lateral perpendicular ao erro (cross-track), nunca lutando
//   contra o avanço ao longo da trajetória.
// - Quando damp_along_error == true (aproximação ao alvo final), amortece em 2D mas faz coast
//   (0, 0) se a frenagem reversa ficar abaixo do piso mínimo, evitando amplificar um pequeno
//   comando negativo em reverso brusco (-min_linear_speed).
inline void computePdLinearVectorWithFloor(
  double ex_m,
  double ey_m,
  double vx_mps,
  double vy_mps,
  double p_gain,
  double d_gain,
  double min_linear,
  double max_linear,
  bool damp_along_error,
  double & out_vx,
  double & out_vy)
{
  out_vx = 0.0;
  out_vy = 0.0;
  if (!std::isfinite(ex_m) || !std::isfinite(ey_m) || !std::isfinite(p_gain) || p_gain <= 0.0) {
    return;
  }
  const double e_norm = std::hypot(ex_m, ey_m);
  if (e_norm <= 1e-9) {
    return;
  }

  const double d_eff = (std::isfinite(d_gain) && d_gain > 0.0) ? d_gain : 0.0;
  if (d_eff <= 0.0) {
    out_vx = p_gain * ex_m;
    out_vy = p_gain * ey_m;
    return;
  }

  const double ux = ex_m / e_norm;
  const double uy = ey_m / e_norm;
  const double vx = std::isfinite(vx_mps) ? vx_mps : 0.0;
  const double vy = std::isfinite(vy_mps) ? vy_mps : 0.0;

  const double v_along = vx * ux + vy * uy;
  const double v_perp_x = vx - v_along * ux;
  const double v_perp_y = vy - v_along * uy;

  const double eff_min = effectiveMinSpeed(min_linear, max_linear);
  const double p_mag = p_gain * e_norm;

  // Amortecimento lateral limitado a 50% do comando base para ruído de velocidade nunca virar loop
  double dx_perp = -d_eff * v_perp_x;
  double dy_perp = -d_eff * v_perp_y;
  const double perp_mag = std::hypot(dx_perp, dy_perp);
  const double max_perp = 0.5 * std::max(p_mag, eff_min);
  if (perp_mag > max_perp && perp_mag > 1e-9) {
    const double s = max_perp / perp_mag;
    dx_perp *= s;
    dy_perp *= s;
  }

  double v_fwd = p_mag - (damp_along_error ? (d_eff * v_along) : 0.0);
  if (eff_min > 0.0 && v_fwd < 0.0 && std::abs(v_fwd) < eff_min) {
    // Zona de coast: não amplificar frenagem pequena para -min_linear_speed nem desviar de lado
    if (std::hypot(dx_perp, dy_perp) < eff_min) {
      out_vx = 0.0;
      out_vy = 0.0;
      return;
    }
    v_fwd = 0.0;
  }

  out_vx = v_fwd * ux + dx_perp;
  out_vy = v_fwd * uy + dy_perp;
}

}  // namespace movement

#endif  // OXEBOTS_STRATEGY__VELOCITY_LIMITS_HPP_
