#pragma once

namespace oxebots_strategy
{

// Histerese compartilhada para decidir se o robô já está "pronto para chutar".
// Usada tanto por UpdateBallPositionNode (simple_attack/goalkeeper_tree) quanto por
// CalculateInterceptionNode (master_strategy), que antes reimplementavam a mesma
// regra de forma independente.
inline bool computeKickReadiness(bool current_ready, double dist_to_pk, double dot)
{
  constexpr double kEnterDistanceMm = 150.0;  // Distância para entrar no estado "pronto".
  constexpr double kStayDistanceMm = 600.0;   // Distância para permanecer "pronto" (histerese).

  const double threshold = current_ready ? kStayDistanceMm : kEnterDistanceMm;
  return (dist_to_pk < threshold) && (dot > 0.0);
}

}  // namespace oxebots_strategy
