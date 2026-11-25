// Testes unitários das funções puras de limites mínimo/máximo de velocidade e cinemática.
// Sem ROS: só a biblioteca padrão e gtest.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "oxebots_strategy/goalkeeper_geometry.hpp"
#include "oxebots_strategy/velocity_limits.hpp"

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
}

TEST(VelocityLimits, SanitizeAndEffectiveMinRespectMaxCap)
{
  EXPECT_DOUBLE_EQ(movement::sanitizeSpeedLimit(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(movement::sanitizeSpeedLimit(kNaN), 0.0);
  EXPECT_DOUBLE_EQ(movement::sanitizeSpeedLimit(kInf), 0.0);
  EXPECT_DOUBLE_EQ(movement::sanitizeSpeedLimit(0.45), 0.45);

  // Precedência do teto máximo (ex.: HALT=0, STOP=0.75, max_speed=0.5)
  EXPECT_DOUBLE_EQ(movement::effectiveMinSpeed(0.3, 1.0), 0.3);
  EXPECT_DOUBLE_EQ(movement::effectiveMinSpeed(0.9, 0.75), 0.75);
  EXPECT_DOUBLE_EQ(movement::effectiveMinSpeed(0.4, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(movement::effectiveMinSpeed(-0.2, 1.0), 0.0);
}

TEST(VelocityLimits, LinearSpeedKeepsZeroAndClampsNonZero)
{
  // Zero permanece zero (HALT, arrived, alinhando no lugar)
  EXPECT_DOUBLE_EQ(movement::applyMinMaxLinearSpeed(0.0, 0.3, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(movement::applyMinMaxLinearSpeed(1e-5, 0.3, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(movement::applyMinMaxLinearSpeed(kNaN, 0.3, 1.0), 0.0);

  // Comando não-nulo abaixo da mínima sobe para a mínima
  EXPECT_DOUBLE_EQ(movement::applyMinMaxLinearSpeed(0.05, 0.3, 1.0), 0.3);
  // Dentro da faixa permanece inalterado
  EXPECT_DOUBLE_EQ(movement::applyMinMaxLinearSpeed(0.6, 0.3, 1.0), 0.6);
  // Acima da máxima cai para a máxima
  EXPECT_DOUBLE_EQ(movement::applyMinMaxLinearSpeed(1.8, 0.3, 1.0), 1.0);
  // Se mínima > máxima, prevalece o teto máximo
  EXPECT_DOUBLE_EQ(movement::applyMinMaxLinearSpeed(0.2, 0.9, 0.5), 0.5);
  // Se máxima == 0 (HALT), zera mesmo com velocidade > 0
  EXPECT_DOUBLE_EQ(movement::applyMinMaxLinearSpeed(0.5, 0.3, 0.0), 0.0);
}

TEST(VelocityLimits, LinearVectorPreservesDirectionAndZero)
{
  float vx = 0.0f;
  float vy = 0.0f;
  movement::applyMinMaxLinearVector(vx, vy, 0.35, 1.0);
  EXPECT_FLOAT_EQ(vx, 0.0f);
  EXPECT_FLOAT_EQ(vy, 0.0f);

  // Vetor (0.03, 0.04) tem norma 0.05 -> deve subir para 0.35 mantendo razão 3:4
  vx = 0.03f;
  vy = 0.04f;
  movement::applyMinMaxLinearVector(vx, vy, 0.35, 1.0);
  EXPECT_NEAR(std::hypot(vx, vy), 0.35, 1e-5);
  EXPECT_NEAR(vx, 0.21f, 1e-5);
  EXPECT_NEAR(vy, 0.28f, 1e-5);

  // Vetor (0.6, -0.8) tem norma 1.0 -> teto 0.75 (STOP) prevalece sobre mínima 0.9
  vx = 0.6f;
  vy = -0.8f;
  movement::applyMinMaxLinearVector(vx, vy, 0.9, 0.75);
  EXPECT_NEAR(std::hypot(vx, vy), 0.75, 1e-5);
  EXPECT_NEAR(vx, 0.45f, 1e-5);
  EXPECT_NEAR(vy, -0.60f, 1e-5);
}

TEST(VelocityLimits, AngularSpeedPreservesSignZeroAndMaxCap)
{
  EXPECT_DOUBLE_EQ(movement::applyMinMaxAngularSpeed(0.0, 0.8, 3.0), 0.0);
  EXPECT_DOUBLE_EQ(movement::applyMinMaxAngularSpeed(1e-5, 0.8, 3.0), 0.0);
  EXPECT_DOUBLE_EQ(movement::applyMinMaxAngularSpeed(-1e-5, 0.8, 3.0), 0.0);
  EXPECT_DOUBLE_EQ(movement::applyMinMaxAngularSpeed(kNaN, 0.8, 3.0), 0.0);

  EXPECT_DOUBLE_EQ(movement::applyMinMaxAngularSpeed(0.2, 0.8, 3.0), 0.8);
  EXPECT_DOUBLE_EQ(movement::applyMinMaxAngularSpeed(-0.2, 0.8, 3.0), -0.8);
  EXPECT_DOUBLE_EQ(movement::applyMinMaxAngularSpeed(1.5, 0.8, 3.0), 1.5);
  EXPECT_DOUBLE_EQ(movement::applyMinMaxAngularSpeed(-5.0, 0.8, 3.0), -3.0);
  EXPECT_DOUBLE_EQ(movement::applyMinMaxAngularSpeed(-0.5, 4.0, 2.0), -2.0);
  EXPECT_DOUBLE_EQ(movement::applyMinMaxAngularSpeed(1.0, 0.8, 0.0), 0.0);
}

TEST(VelocityLimits, AdaptiveDeadbandsStayWithinBounds)
{
  // Com min=0 mantém o valor base exato
  EXPECT_DOUBLE_EQ(movement::effectiveArrivalDistanceMm(10.0, 0.0, 1.0), 10.0);
  EXPECT_DOUBLE_EQ(movement::effectiveAngleToleranceRad(0.04, 0.0, 3.0), 0.04);

  // Com min baixo (0.3 m/s -> 6 mm em 20 ms) continua no base (10 mm)
  EXPECT_DOUBLE_EQ(movement::effectiveArrivalDistanceMm(10.0, 0.3, 1.0), 10.0);
  // Com min alto (0.8 m/s -> 16 mm em 20 ms) cresce para absorver 1 ciclo
  EXPECT_DOUBLE_EQ(movement::effectiveArrivalDistanceMm(10.0, 0.8, 1.0), 16.0);
  // Nunca ultrapassa o teto de segurança (25 mm, abaixo da menor tolerância de 50 mm das BTs)
  EXPECT_DOUBLE_EQ(movement::effectiveArrivalDistanceMm(10.0, 2.0, 2.0), 25.0);

  // Angular: min=6 rad/s -> 0.5*6*0.02 = 0.06 rad; limitado a 0.12 rad (< 0.15 rad do GoToPoint)
  EXPECT_DOUBLE_EQ(movement::effectiveAngleToleranceRad(0.04, 6.0, 8.0), 0.06);
  EXPECT_DOUBLE_EQ(movement::effectiveAngleToleranceRad(0.04, 20.0, 20.0), 0.12);
}

TEST(VelocityLimits, EstimateTravelTimeMatchesClassicAndAdaptsToMinSpeed)
{
  const double accel = 2000.0;
  const double v_max = 1500.0;

  // Caso clássico (min=0, v0=0, distância longa = trapezoidal)
  // d_acc = d_dec = 1500^2 / 4000 = 562.5 mm; d_total = 2000 mm
  // t = 1500/2000 + 1500/2000 + (2000 - 1125)/1500 = 0.75 + 0.75 + 875/1500
  const double t_classic = movement::estimateTravelTimeS(2000.0, 0.0, 0.0, v_max, accel);
  EXPECT_NEAR(t_classic, 1.5 + 875.0 / 1500.0, 1e-9);

  // Com min_speed > 0, não precisa desacelerar até 0 na chegada -> chega mais rápido (ou igual)
  const double t_with_min = movement::estimateTravelTimeS(2000.0, 0.0, 600.0, v_max, accel);
  EXPECT_GT(t_with_min, 0.0);
  EXPECT_LT(t_with_min, t_classic);

  // Se min_speed == max_speed e já partindo a v_max -> t = dist / v_max
  const double t_const = movement::estimateTravelTimeS(1500.0, v_max, v_max, v_max, accel);
  EXPECT_NEAR(t_const, 1.0, 1e-9);

  // Se min_speed > max_speed -> clampeado a max_speed, sem falhar nem retornar NaN
  const double t_clamped = movement::estimateTravelTimeS(1500.0, v_max, 3000.0, v_max, accel);
  EXPECT_NEAR(t_clamped, 1.0, 1e-9);

  // Velocidade inicial contrária (v0 < 0): inclui frenagem + deslocamento
  const double t_reverse = movement::estimateTravelTimeS(1000.0, -500.0, 300.0, v_max, accel);
  EXPECT_TRUE(std::isfinite(t_reverse));
  EXPECT_GT(t_reverse, movement::estimateTravelTimeS(1000.0, 0.0, 300.0, v_max, accel));

  // Entradas degeneradas nunca produzem NaN/Inf
  EXPECT_DOUBLE_EQ(movement::estimateTravelTimeS(0.0, 0.0, 300.0, v_max, accel), 0.0);
  EXPECT_DOUBLE_EQ(movement::estimateTravelTimeS(-10.0, 0.0, 300.0, v_max, accel), 0.0);
  EXPECT_DOUBLE_EQ(movement::estimateTravelTimeS(500.0, 0.0, 300.0, 0.0, accel), 0.0);
}

TEST(VelocityLimits, GoalkeeperReachTimeHandlesZeroSpeedSafely)
{
  namespace gk = oxebots_strategy::goalkeeper;
  EXPECT_TRUE(std::isinf(gk::reachTimeS({0.0, 0.0}, {1000.0, 0.0}, 0.0, 90.0)));
  EXPECT_DOUBLE_EQ(gk::reachTimeS({0.0, 0.0}, {50.0, 0.0}, 0.0, 90.0), 0.0);
  EXPECT_NEAR(gk::reachTimeS({0.0, 0.0}, {1090.0, 0.0}, 1.0, 90.0), 1.0, 1e-9);
}

TEST(VelocityLimits, ProjectPredictedPoseAppliesLatencyWrapAndClamp)
{
  double x = -850.0;
  double y = 100.0;
  double th = 3.05;  // perto de +pi
  // dt_since = 20ms + latency = 50ms -> total 70ms
  movement::projectPredictedPoseMm(x, y, th, 1.0, -0.5, 3.0, 0.02, 0.05);
  EXPECT_NEAR(x, -850.0 + 70.0, 1e-6);
  EXPECT_NEAR(y, 100.0 - 35.0, 1e-6);
  EXPECT_NEAR(th, 3.05 + 0.21 - 2.0 * M_PI, 1e-6);

  // Horizonte excessivo clampeado em max_horizon_s (0.20s)
  double x2 = 0.0, y2 = 0.0, th2 = 0.0;
  movement::projectPredictedPoseMm(x2, y2, th2, 1.0, 0.0, 0.0, 5.0, 5.0, 0.20);
  EXPECT_NEAR(x2, 200.0, 1e-6);
}

TEST(VelocityLimits, PdAngularWithFloorPreventsReverseChatter)
{
  // angle_err=+0.18 rad, p=2.2, omega=+1.4 rad/s, d=0.45 -> pd = 0.396 - 0.63 = -0.234
  // Como |-0.234| < min_ang (1.40) e tem sinal oposto ao erro, deve fazer coast (0.0),
  // nunca amplificar para -1.40 rad/s!
  EXPECT_DOUBLE_EQ(
    movement::applyPdAngularSpeedWithFloor(0.18, 1.4, 2.2, 0.45, 1.40, 4.0), 0.0);

  // Overshoot extremo (pd = 0.22 - 2.25 = -2.03, módulo >= 1.40): frenagem reversa permitida
  EXPECT_NEAR(
    movement::applyPdAngularSpeedWithFloor(0.10, 5.0, 2.2, 0.45, 1.40, 4.0), -2.03, 1e-6);

  // Erro no mesmo sentido do pd (sem inversão): aplica piso mínimo normalmente
  EXPECT_DOUBLE_EQ(
    movement::applyPdAngularSpeedWithFloor(0.20, 0.0, 2.2, 0.25, 1.40, 4.0), 1.40);
}

TEST(VelocityLimits, PdLinearWithFloorCrossTrackAndCoast)
{
  double vx = 0.0, vy = 0.0;

  // Seguindo lookahead móvel (damp_along_error=false): não freia avanço em +Y, só amortece lateral em X
  movement::computePdLinearVectorWithFloor(
    0.0, 0.14, -0.4, 1.0, 2.5, 0.15, 0.35, 1.0, false, vx, vy);
  EXPECT_NEAR(vy, 0.35, 1e-6);
  EXPECT_NEAR(vx, 0.06, 1e-6);

  // Aproximação final (damp_along_error=true): frenagem leve (-0.025 m/s) abaixo do piso (0.35) -> coast (0, 0)
  movement::computePdLinearVectorWithFloor(
    0.0, 0.05, 0.0, 1.0, 2.5, 0.15, 0.35, 1.0, true, vx, vy);
  EXPECT_DOUBLE_EQ(vx, 0.0);
  EXPECT_DOUBLE_EQ(vy, 0.0);

  // d_gain = 0: equivalente ao P puro
  movement::computePdLinearVectorWithFloor(
    0.1, 0.2, 1.0, 1.0, 2.5, 0.0, 0.35, 1.0, true, vx, vy);
  EXPECT_NEAR(vx, 0.25, 1e-6);
  EXPECT_NEAR(vy, 0.50, 1e-6);
}
