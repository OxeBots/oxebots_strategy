// Geometria pura do goleiro: segurança de passe (cápsula/cone) e escolha da direção de afastar.
//
// Só usa a biblioteca padrão (sem ROS nem BehaviorTree.CPP), para poder ser testada isoladamente
// (test/test_goalkeeper_geometry.cpp). Unidades: mm e radianos, no mesmo referencial do campo
// usado pelo restante da estratégia.
#ifndef OXEBOTS_STRATEGY__GOALKEEPER_GEOMETRY_HPP_
#define OXEBOTS_STRATEGY__GOALKEEPER_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace oxebots_strategy::goalkeeper
{

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

inline double distance(const Vec2 & a, const Vec2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

// Parâmetros do corredor de passe: raio do robô (90) + raio da bola (~21) + folga, que cresce com a
// distância (erro de pontaria e movimento do adversário), e pontas maiores em volta da bola e do aliado.
struct PassParams
{
  double corridor_base_mm = 170.0;
  double corridor_growth = 0.08;
  double ball_cap_mm = 300.0;   // adversário colado na bola/goleiro: sem chute limpo
  double ally_cap_mm = 350.0;   // adversário colado no receptor: ele seria pressionado ao receber
};

// Projeção de um ponto no segmento a->b: "along" é a distância ao longo do segmento a partir de a (pode
// ser negativa ou maior que o comprimento) e "perp" a distância perpendicular à reta (sempre >= 0).
struct SegmentProjection
{
  double along;
  double perp;
};

inline SegmentProjection projectOnSegment(const Vec2 & a, const Vec2 & b, const Vec2 & p)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-9) {
    return {0.0, distance(a, p)};
  }
  const double ux = dx / len;
  const double uy = dy / len;
  const double rx = p.x - a.x;
  const double ry = p.y - a.y;
  return {rx * ux + ry * uy, std::abs(rx * uy - ry * ux)};
}

// Distância de um ponto ao segmento (inclusive às pontas).
inline double distPointSegment(const Vec2 & a, const Vec2 & b, const Vec2 & p)
{
  const double len = distance(a, b);
  const SegmentProjection pr = projectOnSegment(a, b, p);
  if (pr.along <= 0.0) {
    return distance(a, p);
  }
  if (pr.along >= len) {
    return distance(b, p);
  }
  return pr.perp;
}

// Folga do passe bola->aliado em relação aos adversários, em mm: o menor valor de
// (distância do adversário - raio exigido) entre
//  - o corredor (segmento engrossado por corridor_base + corridor_growth * distância percorrida),
//  - a ponta da bola (ball_cap) e
//  - a ponta do aliado (ally_cap).
// Positiva = passe livre; negativa = há adversário dentro da região proibida. Sem adversários: +infinito.
inline double passClearance(
  const Vec2 & ball, const Vec2 & ally, const std::vector<Vec2> & opponents,
  const PassParams & params = PassParams())
{
  double clearance = std::numeric_limits<double>::infinity();
  const double len = distance(ball, ally);
  for (const Vec2 & opp : opponents) {
    clearance = std::min(clearance, distance(opp, ball) - params.ball_cap_mm);
    clearance = std::min(clearance, distance(opp, ally) - params.ally_cap_mm);
    const SegmentProjection pr = projectOnSegment(ball, ally, opp);
    if (pr.along > 0.0 && pr.along < len) {
      const double radius = params.corridor_base_mm + params.corridor_growth * pr.along;
      clearance = std::min(clearance, pr.perp - radius);
    }
  }
  return clearance;
}

// Folga de um raio de chute (bola + direção) em relação aos adversários, no mesmo corredor do passe mas
// só à frente da bola e sem a ponta do receptor.
inline double rayClearance(
  const Vec2 & ball, double angle, double length_mm, const std::vector<Vec2> & opponents,
  const PassParams & params = PassParams())
{
  const Vec2 end{ball.x + std::cos(angle) * length_mm, ball.y + std::sin(angle) * length_mm};
  double clearance = std::numeric_limits<double>::infinity();
  for (const Vec2 & opp : opponents) {
    const SegmentProjection pr = projectOnSegment(ball, end, opp);
    if (pr.along <= 0.0 || pr.along >= length_mm) {
      continue;
    }
    const double radius = params.corridor_base_mm + params.corridor_growth * pr.along;
    clearance = std::min(clearance, pr.perp - radius);
  }
  return clearance;
}

// O raio de chute sai do campo antes de "length_mm"? (campo centrado na origem, meio-comprimento
// half_len_mm e meia-largura half_wid_mm, com uma margem interna.)
inline bool rayLeavesField(
  const Vec2 & ball, double angle, double length_mm, double half_len_mm, double half_wid_mm,
  double margin_mm = 50.0)
{
  const double ex = ball.x + std::cos(angle) * length_mm;
  const double ey = ball.y + std::sin(angle) * length_mm;
  return std::abs(ex) > half_len_mm - margin_mm || std::abs(ey) > half_wid_mm - margin_mm;
}

// Distância (mm) da bola até a fronteira do campo seguindo "angle" (campo centrado na origem, meio-comprimento
// half_len_mm e meia-largura half_wid_mm). Se a bola já está fora, devolve 0.
inline double rayDistanceToField(
  const Vec2 & ball, double angle, double half_len_mm, double half_wid_mm)
{
  const double ux = std::cos(angle);
  const double uy = std::sin(angle);
  double t = std::numeric_limits<double>::infinity();
  if (ux > 1e-9) {
    t = std::min(t, (half_len_mm - ball.x) / ux);
  } else if (ux < -1e-9) {
    t = std::min(t, (-half_len_mm - ball.x) / ux);
  }
  if (uy > 1e-9) {
    t = std::min(t, (half_wid_mm - ball.y) / uy);
  } else if (uy < -1e-9) {
    t = std::min(t, (-half_wid_mm - ball.y) / uy);
  }
  return std::max(0.0, t);
}

// Velocidade de chute (m/s) para a bola, com atrito de rolamento constante, parar depois de "distance_mm":
// v = sqrt(2 * a * d). O resultado é limitado a [min_mps, max_mps].
inline double kickSpeedToStopWithin(
  double distance_mm, double friction_mps2, double min_mps, double max_mps)
{
  const double v = std::sqrt(2.0 * friction_mps2 * std::max(0.0, distance_mm) / 1000.0);
  return std::clamp(v, min_mps, max_mps);
}

struct FanParams
{
  double half_span_rad = 60.0 * M_PI / 180.0;   // abertura do leque em volta da direção "para a frente"
  double step_rad = 10.0 * M_PI / 180.0;
  double ray_length_mm = 1200.0;                // trecho do chute avaliado quanto a adversários
  double exit_check_mm = 1000.0;                // trecho avaliado quanto a sair do campo
  double clearance_cap_mm = 600.0;              // folga acima disso não melhora o escore
  double exit_penalty = 0.25;                   // sair do campo é aceitável, mas só se nada melhor existir
  double center_weight = 0.2;                   // preferência por manter a direção para a frente
  double advantage_weight = 0.6;                // peso da vantagem de posse no ponto onde a bola para
};

struct FanChoice
{
  double angle;        // direção escolhida (rad)
  double score;
  double clearance;    // folga do raio escolhido (mm, +infinito se não há adversários)
  bool leaves_field;
  double speed = 0.0;  // velocidade de chute escolhida (m/s), quando a escolha considera o destino da bola
};

// Escolhe, entre as direções do leque, a de maior escore: folga de adversários (normalizada em
// [-1, 1]), menos uma penalidade se a bola sai do campo, mais uma preferência por "para a frente".
inline FanChoice chooseClearAngle(
  const Vec2 & ball, double forward_angle, const std::vector<Vec2> & opponents,
  double half_len_mm, double half_wid_mm, const PassParams & pass = PassParams(),
  const FanParams & fan = FanParams())
{
  FanChoice best{forward_angle, -std::numeric_limits<double>::infinity(), 0.0, false};
  const int steps = static_cast<int>(std::floor(fan.half_span_rad / fan.step_rad + 1e-9));
  for (int i = -steps; i <= steps; ++i) {
    const double offset = i * fan.step_rad;
    const double angle = forward_angle + offset;
    const double clearance = rayClearance(ball, angle, fan.ray_length_mm, opponents, pass);
    const double norm = std::clamp(clearance / fan.clearance_cap_mm, -1.0, 1.0);
    const bool leaves = rayLeavesField(ball, angle, fan.exit_check_mm, half_len_mm, half_wid_mm);
    const double score = norm - (leaves ? fan.exit_penalty : 0.0) + fan.center_weight * std::cos(
      offset);
    // Empate: fica com a direção mais próxima de "para a frente" (a varredura já passa por ela no meio).
    const bool better = score > best.score + 1e-9 ||
      (std::abs(score - best.score) <= 1e-9 &&
      std::abs(offset) < std::abs(best.angle - forward_angle));
    if (better) {
      best = {angle, score, clearance, leaves};
    }
  }
  return best;
}

// Escore de um passe seguro (maior = melhor): folga (normalizada), progresso para o campo adversário e
// distância (passes longos são menos precisos). "forward_sign" é +1 se o ataque é para +x e -1 caso contrário.
inline double passScore(
  const Vec2 & ball, const Vec2 & ally, double clearance_mm, double forward_sign,
  double half_wid_mm, double max_pass_mm = 2500.0)
{
  const double clearance_norm = std::clamp(clearance_mm / 400.0, 0.0, 1.0);
  const double progress = std::clamp(forward_sign * (ally.x - ball.x) / 2000.0, 0.0, 1.0);
  const double dist_norm = std::clamp(distance(ball, ally) / max_pass_mm, 0.0, 1.0);
  const double sideline_penalty = (std::abs(ally.y) > half_wid_mm - 300.0) ? 0.2 : 0.0;
  return 0.5 * clearance_norm + 0.35 * progress - 0.15 * dist_norm - sideline_penalty;
}

// ---------------------------------------------------------------------------------------------------------------
// Destino da bola e posse
// ---------------------------------------------------------------------------------------------------------------

struct DestinationParams
{
  double ally_speed_mps = 1.0;     // velocidade de aproximação dos nossos robôs (max_linear_speed)
  double enemy_speed_mps = 1.3;    // adversários: conservador (o regulamento limita a 1,5 m/s)
  double robot_radius_mm = 90.0;
  double friction_mps2 = 0.8;      // atrito de rolamento da bola (o mesmo do filtro de Kalman)
  double time_cap_s = 3.0;         // diferença de tempo que satura a vantagem de posse
};

// Distância percorrida por uma bola chutada a "speed_mps" até parar, com atrito constante.
inline double ballStopDistanceMm(double speed_mps, double friction_mps2)
{
  return speed_mps * speed_mps / (2.0 * friction_mps2) * 1000.0;
}

struct Landing
{
  Vec2 point;         // onde a bola para (ou onde cruza a borda do campo)
  double travel_mm;
  bool leaves_field;  // a bola sai do campo antes de parar
};

inline Landing landingPoint(
  const Vec2 & ball, double angle, double speed_mps, double friction_mps2,
  double half_len_mm, double half_wid_mm)
{
  const double stop = ballStopDistanceMm(speed_mps, friction_mps2);
  const double edge = rayDistanceToField(ball, angle, half_len_mm, half_wid_mm);
  const double travel = std::min(stop, edge);
  return {{ball.x + std::cos(angle) * travel, ball.y + std::sin(angle) * travel}, travel,
    stop > edge};
}

// Tempo (s) para um robô chegar ao ponto (descontado o raio do robô).
inline double reachTimeS(
  const Vec2 & robot, const Vec2 & point, double speed_mps, double radius_mm)
{
  return std::max(0.0, distance(robot, point) - radius_mm) / 1000.0 / speed_mps;
}

// Vantagem de posse no ponto, em [-1, 1]: positiva quando algum dos nossos chega antes de qualquer adversário.
// Sem adversários em campo vale 1; sem aliados ativos, é só um sinal fraco (+0,5 no máximo) de quanto o
// adversário demora.
inline double possessionAdvantage(
  const Vec2 & point, const std::vector<Vec2> & allies, const std::vector<Vec2> & enemies,
  const DestinationParams & params = DestinationParams())
{
  double t_enemy = std::numeric_limits<double>::infinity();
  double t_ally = std::numeric_limits<double>::infinity();
  for (const Vec2 & e : enemies) {
    t_enemy =
      std::min(t_enemy, reachTimeS(e, point, params.enemy_speed_mps, params.robot_radius_mm));
  }
  for (const Vec2 & a : allies) {
    t_ally = std::min(t_ally, reachTimeS(a, point, params.ally_speed_mps, params.robot_radius_mm));
  }
  if (std::isinf(t_enemy)) {
    return 1.0;
  }
  if (std::isinf(t_ally)) {
    return 0.5 * std::clamp(t_enemy / params.time_cap_s, 0.0, 1.0);
  }
  return std::clamp((t_enemy - t_ally) / params.time_cap_s, -1.0, 1.0);
}

// Como chooseClearAngle, mas também pontua a vantagem de posse no ponto onde a bola para. Com "keep_in_field"
// a velocidade do chute é limitada em cada direção para a bola parar a "stop_margin_mm" da borda (posse nossa:
// não mandar a bola para fora); sem ele o chute é de "max_speed_mps" (afastar com força).
inline FanChoice chooseClearAngleWithPossession(
  const Vec2 & ball, double forward_angle, double max_speed_mps, bool keep_in_field,
  const std::vector<Vec2> & allies, const std::vector<Vec2> & enemies,
  double half_len_mm, double half_wid_mm, const PassParams & pass = PassParams(),
  const FanParams & fan = FanParams(), const DestinationParams & dest = DestinationParams(),
  double stop_margin_mm = 400.0, double min_speed_mps = 1.0,
  double previous_angle = std::numeric_limits<double>::quiet_NaN(), double stickiness = 0.0)
{
  FanChoice best{forward_angle, -std::numeric_limits<double>::infinity(), 0.0, false,
    max_speed_mps};
  const int steps = static_cast<int>(std::floor(fan.half_span_rad / fan.step_rad + 1e-9));
  for (int i = -steps; i <= steps; ++i) {
    const double offset = i * fan.step_rad;
    const double angle = forward_angle + offset;
    const double clearance = rayClearance(ball, angle, fan.ray_length_mm, enemies, pass);
    const double norm = std::clamp(clearance / fan.clearance_cap_mm, -1.0, 1.0);

    double speed = max_speed_mps;
    if (keep_in_field) {
      const double room =
        rayDistanceToField(ball, angle, half_len_mm, half_wid_mm) - stop_margin_mm;
      speed = kickSpeedToStopWithin(room, dest.friction_mps2, min_speed_mps, max_speed_mps);
    }
    const Landing landing = landingPoint(
      ball, angle, speed, dest.friction_mps2, half_len_mm,
      half_wid_mm);
    const double advantage = possessionAdvantage(landing.point, allies, enemies, dest);

    // Estabilidade: a direção escolhida no tick anterior ganha um bônus, para o robô não ficar trocando de
    // lado da bola quando duas direções têm escores quase iguais.
    const bool same_as_previous = !std::isnan(previous_angle) &&
      std::abs(std::remainder(angle - previous_angle, 2.0 * M_PI)) < fan.step_rad * 0.6;
    const double score = norm + fan.advantage_weight * advantage -
      (landing.leaves_field ? fan.exit_penalty : 0.0) + fan.center_weight * std::cos(offset) +
      (same_as_previous ? stickiness : 0.0);
    const bool better = score > best.score + 1e-9 ||
      (std::abs(score - best.score) <= 1e-9 &&
      std::abs(offset) < std::abs(best.angle - forward_angle));
    if (better) {
      best = {angle, score, clearance, landing.leaves_field, speed};
    }
  }
  return best;
}

// ---------------------------------------------------------------------------------------------------------------
// Ameaça de chute e caminho até a bola
// ---------------------------------------------------------------------------------------------------------------

struct ThreatParams
{
  double ball_radius_mm = 600.0;   // adversário perto da bola o bastante para chutar logo
  double align_rad = 0.6;          // erro máximo entre a orientação dele e a direção do nosso gol
};

// Chute iminente contra o nosso gol: um adversário perto da bola, com a bola entre ele e o nosso gol e
// apontando (aproximadamente) para o gol.
inline bool isShotThreat(
  const Vec2 & opponent, double opponent_heading, const Vec2 & ball, const Vec2 & goal_center,
  const ThreatParams & params = ThreatParams())
{
  if (distance(opponent, ball) > params.ball_radius_mm) {
    return false;
  }
  const double between = (ball.x - opponent.x) * (goal_center.x - ball.x) +
    (ball.y - opponent.y) * (goal_center.y - ball.y);
  if (between <= 0.0) {
    return false;
  }
  const double to_goal = std::atan2(goal_center.y - opponent.y, goal_center.x - opponent.x);
  return std::abs(std::remainder(opponent_heading - to_goal, 2.0 * M_PI)) < params.align_rad;
}

// O caminho do robô até a bola está bloqueado por um adversário? O trecho final (ball_zone_mm antes da bola) não
// conta: o adversário que disputa a bola está sempre ali. corridor_mm = raio dos dois robôs + folga.
inline bool approachBlocked(
  const Vec2 & robot, const Vec2 & ball, const std::vector<Vec2> & enemies,
  double corridor_mm = 250.0, double ball_zone_mm = 350.0)
{
  const double len = distance(robot, ball);
  if (len <= ball_zone_mm) {
    return false;
  }
  const double k = (len - ball_zone_mm) / len;
  const Vec2 end{robot.x + (ball.x - robot.x) * k, robot.y + (ball.y - robot.y) * k};
  for (const Vec2 & e : enemies) {
    if (distPointSegment(robot, end, e) < corridor_mm) {
      return true;
    }
  }
  return false;
}

}  // namespace oxebots_strategy::goalkeeper

#endif  // OXEBOTS_STRATEGY__GOALKEEPER_GEOMETRY_HPP_
