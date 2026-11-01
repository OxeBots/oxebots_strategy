// Testes das funções puras de geometria do goleiro (passe com corredor/cone e leque de afastar).
// Sem ROS: só a biblioteca padrão e gtest.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "oxebots_strategy/goalkeeper_geometry.hpp"

namespace gk = oxebots_strategy::goalkeeper;

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
}

TEST(SegmentGeometry, ProjecaoEDistanciaAoSegmento)
{
  const gk::Vec2 a{0.0, 0.0};
  const gk::Vec2 b{1000.0, 0.0};

  const auto meio = gk::projectOnSegment(a, b, {400.0, 150.0});
  EXPECT_NEAR(meio.along, 400.0, 1e-9);
  EXPECT_NEAR(meio.perp, 150.0, 1e-9);

  // Antes de "a" e depois de "b" a distância ao segmento é a distância à ponta.
  EXPECT_NEAR(gk::distPointSegment(a, b, {-300.0, 400.0}), 500.0, 1e-9);
  EXPECT_NEAR(gk::distPointSegment(a, b, {1300.0, 400.0}), 500.0, 1e-9);
  EXPECT_NEAR(gk::distPointSegment(a, b, {500.0, -200.0}), 200.0, 1e-9);

  // Segmento degenerado (a == b): distância ao ponto.
  EXPECT_NEAR(gk::distPointSegment(a, a, {30.0, 40.0}), 50.0, 1e-9);
}

TEST(PassClearance, SemAdversariosELivre)
{
  EXPECT_EQ(gk::passClearance({0.0, 0.0}, {1000.0, 0.0}, {}), kInf);
}

TEST(PassClearance, AdversarioNaLinhaBloqueia)
{
  const double c = gk::passClearance({0.0, 0.0}, {1500.0, 0.0}, {{700.0, 0.0}});
  EXPECT_LT(c, 0.0);
}

TEST(PassClearance, AdversarioAoLadoForaDoCorredorLibera)
{
  // A 700 mm ao longo do passe o raio exigido é 170 + 0,08*700 = 226 mm; a 400 mm da linha está livre.
  const double c = gk::passClearance({0.0, 0.0}, {1500.0, 0.0}, {{700.0, 400.0}});
  EXPECT_GT(c, 0.0);
  EXPECT_NEAR(c, 400.0 - (170.0 + 0.08 * 700.0), 1e-9);
}

TEST(PassClearance, CorredorCresceComADistancia)
{
  // Mesmo afastamento lateral (300 mm): perto da bola passa, perto do aliado bloqueia.
  const gk::Vec2 ball{0.0, 0.0};
  const gk::Vec2 ally{2500.0, 0.0};
  const double perto = gk::passClearance(ball, ally, {{800.0, 300.0}});
  const double longe = gk::passClearance(ball, ally, {{1800.0, 300.0}});
  EXPECT_GT(perto, 0.0);    // raio 170 + 64 = 234 < 300
  EXPECT_LT(longe, 0.0);    // raio 170 + 144 = 314 > 300
}

TEST(PassClearance, AdversarioAtrasDaBolaMuitoPertoBloqueia)
{
  // Atrás da bola, a 200 mm: dentro da ponta da bola (300 mm), mesmo fora do corredor.
  const double c = gk::passClearance({0.0, 0.0}, {1500.0, 0.0}, {{-200.0, 0.0}});
  EXPECT_LT(c, 0.0);
}

TEST(PassClearance, AdversarioAtrasDaBolaLongeNaoImporta)
{
  EXPECT_GT(gk::passClearance({0.0, 0.0}, {1500.0, 0.0}, {{-600.0, 0.0}}), 0.0);
}

TEST(PassClearance, AdversarioColadoNoReceptorBloqueia)
{
  // Fora do corredor, mas a 250 mm do aliado (ponta do receptor = 350 mm), na frente dele.
  const double c = gk::passClearance({0.0, 0.0}, {1500.0, 0.0}, {{1700.0, 150.0}});
  EXPECT_LT(c, 0.0);
}

TEST(PassClearance, UsaOMenorValorEntreVariosAdversarios)
{
  const double a = gk::passClearance({0.0, 0.0}, {1500.0, 0.0}, {{700.0, 500.0}});
  const double ab = gk::passClearance({0.0, 0.0}, {1500.0, 0.0}, {{700.0, 500.0}, {900.0, 100.0}});
  EXPECT_GT(a, 0.0);
  EXPECT_LT(ab, 0.0);
}

TEST(RayLeavesField, DetectaSaidaPelaLateralEPelaLinhaDeFundo)
{
  // Campo de 4400 x 3000 (meio-comprimento 2200, meia-largura 1500).
  EXPECT_FALSE(gk::rayLeavesField({0.0, 0.0}, 0.0, 1000.0, 2200.0, 1500.0));
  EXPECT_TRUE(gk::rayLeavesField({0.0, 1000.0}, M_PI / 2.0, 1000.0, 2200.0, 1500.0));
  EXPECT_TRUE(gk::rayLeavesField({1500.0, 0.0}, 0.0, 1000.0, 2200.0, 1500.0));
}

TEST(ChooseClearAngle, SemAdversariosFicaParaAFrente)
{
  const auto c = gk::chooseClearAngle({-1700.0, 0.0}, 0.0, {}, 2200.0, 1500.0);
  EXPECT_NEAR(c.angle, 0.0, 1e-9);
  EXPECT_FALSE(c.leaves_field);
}

TEST(ChooseClearAngle, DesviaDeAdversarioNaFrente)
{
  // Adversário bem na frente da bola: o leque tem de escolher um ângulo com folga positiva.
  const gk::Vec2 ball{-1700.0, 0.0};
  const auto c = gk::chooseClearAngle(ball, 0.0, {{-1300.0, 0.0}}, 2200.0, 1500.0);
  EXPECT_GT(std::abs(c.angle), 0.2);
  EXPECT_GT(c.clearance, 0.0);
}

TEST(ChooseClearAngle, PrefereManterABolaEmJogo)
{
  // Bola colada na lateral: o "para a frente" inclinado para fora sai do campo; o leque deve preferir
  // uma direção que não saia, se existir.
  const gk::Vec2 ball{-1700.0, 1400.0};
  const double forward = 25.0 * M_PI / 180.0;   // inclinado para a lateral, sai do campo
  const auto c = gk::chooseClearAngle(ball, forward, {}, 2200.0, 1500.0);
  EXPECT_FALSE(c.leaves_field);
  EXPECT_LT(c.angle, forward);
}

TEST(PassScore, PrefereMaisFolgaEMaisProgresso)
{
  const gk::Vec2 ball{-1700.0, 0.0};
  const double folgado = gk::passScore(ball, {-700.0, 300.0}, 400.0, 1.0, 1500.0);
  const double apertado = gk::passScore(ball, {-700.0, 300.0}, 50.0, 1.0, 1500.0);
  const double recuado = gk::passScore(ball, {-1900.0, 300.0}, 400.0, 1.0, 1500.0);
  EXPECT_GT(folgado, apertado);
  EXPECT_GT(folgado, recuado);
}

TEST(RayDistanceToField, DistanciaAteACadaBorda)
{
  // Campo de 4400 x 3000 (meio-comprimento 2200, meia-largura 1500).
  EXPECT_NEAR(gk::rayDistanceToField({0.0, 0.0}, 0.0, 2200.0, 1500.0), 2200.0, 1e-6);
  EXPECT_NEAR(gk::rayDistanceToField({0.0, 0.0}, M_PI / 2.0, 2200.0, 1500.0), 1500.0, 1e-6);
  EXPECT_NEAR(gk::rayDistanceToField({-1000.0, 0.0}, M_PI, 2200.0, 1500.0), 1200.0, 1e-6);
  // Na diagonal, vale a borda que vier primeiro.
  EXPECT_NEAR(
    gk::rayDistanceToField({0.0, 1000.0}, M_PI / 4.0, 2200.0, 1500.0),
    500.0 * std::sqrt(2.0), 1e-6);
  // Bola já fora do campo: distância zero.
  EXPECT_NEAR(gk::rayDistanceToField({2500.0, 0.0}, 0.0, 2200.0, 1500.0), 0.0, 1e-9);
}

TEST(KickSpeedToStopWithin, VelocidadePeloAtrito)
{
  // v = sqrt(2 * 0,8 * 1,0 m) ~ 1,265 m/s
  EXPECT_NEAR(gk::kickSpeedToStopWithin(1000.0, 0.8, 0.0, 3.0), std::sqrt(1.6), 1e-9);
  // Limites mínimo e máximo
  EXPECT_NEAR(gk::kickSpeedToStopWithin(10.0, 0.8, 1.0, 3.0), 1.0, 1e-9);
  EXPECT_NEAR(gk::kickSpeedToStopWithin(20000.0, 0.8, 1.0, 3.0), 3.0, 1e-9);
  // Distância negativa não gera NaN
  EXPECT_NEAR(gk::kickSpeedToStopWithin(-50.0, 0.8, 1.0, 3.0), 1.0, 1e-9);
}

TEST(Destination, DistanciaDeParadaEPontoDeParada)
{
  // 3 m/s com atrito de 0,8 m/s^2 percorre 5,625 m; 2 m/s, 2,5 m.
  EXPECT_NEAR(gk::ballStopDistanceMm(3.0, 0.8), 5625.0, 1e-6);
  EXPECT_NEAR(gk::ballStopDistanceMm(2.0, 0.8), 2500.0, 1e-6);

  // A 2 m/s a partir do centro, para dentro do campo (2200 mm de meio-comprimento).
  const auto dentro = gk::landingPoint({0.0, 0.0}, 0.0, 2.0, 0.8, 2200.0, 1500.0);
  EXPECT_TRUE(dentro.leaves_field);   // 2500 mm > 2200 mm: cruza a linha de fundo
  EXPECT_NEAR(dentro.point.x, 2200.0, 1e-6);

  const auto curto = gk::landingPoint({0.0, 0.0}, 0.0, 1.0, 0.8, 2200.0, 1500.0);
  EXPECT_FALSE(curto.leaves_field);
  EXPECT_NEAR(curto.point.x, 625.0, 1e-6);
}

TEST(Destination, VantagemDePosse)
{
  const gk::Vec2 ponto{0.0, 0.0};
  // Aliado perto, adversário longe: favorece a gente.
  EXPECT_GT(gk::possessionAdvantage(ponto, {{500.0, 0.0}}, {{2000.0, 0.0}}), 0.0);
  // Adversário perto, aliado longe: favorece eles.
  EXPECT_LT(gk::possessionAdvantage(ponto, {{2000.0, 0.0}}, {{500.0, 0.0}}), 0.0);
  // Sem adversários: vantagem máxima.
  EXPECT_NEAR(gk::possessionAdvantage(ponto, {{500.0, 0.0}}, {}), 1.0, 1e-9);
  // Sem aliados: só um sinal fraco, nunca acima de 0,5.
  const double sem_aliados = gk::possessionAdvantage(ponto, {}, {{2000.0, 0.0}});
  EXPECT_GT(sem_aliados, 0.0);
  EXPECT_LE(sem_aliados, 0.5);
}

TEST(ChooseClearAngleWithPossession, PrefereODestinoOndeOsNossosChegamPrimeiro)
{
  // Bola no nosso terço, atacando para +x. Aliado no alto (y > 0) e adversário embaixo (y < 0), os dois à frente.
  const gk::Vec2 ball{-1500.0, 0.0};
  const std::vector<gk::Vec2> aliados{{0.0, 900.0}};
  const std::vector<gk::Vec2> adversarios{{0.0, -900.0}};
  const auto c = gk::chooseClearAngleWithPossession(
    ball, 0.0, 3.0, true, aliados, adversarios, 2200.0, 1500.0);
  EXPECT_GT(c.angle, 0.0);   // vai para o lado do aliado
}

TEST(ChooseClearAngleWithPossession, ComPosseAVelocidadeFazABolaPararDentroDoCampo)
{
  const gk::Vec2 ball{-1700.0, 1000.0};
  const auto c = gk::chooseClearAngleWithPossession(
    ball, 20.0 * M_PI / 180.0, 3.0, true, {{-500.0, 0.0}}, {}, 2200.0, 1500.0);
  EXPECT_LT(c.speed, 3.0);
  const auto land = gk::landingPoint(ball, c.angle, c.speed, 0.8, 2200.0, 1500.0);
  EXPECT_FALSE(land.leaves_field);
}

TEST(ChooseClearAngleWithPossession, AfastarComForcaMantemAVelocidadeMaxima)
{
  const auto c = gk::chooseClearAngleWithPossession(
    {-1700.0, 0.0}, 0.0, 3.0, false, {}, {}, 2200.0, 1500.0);
  EXPECT_NEAR(c.speed, 3.0, 1e-9);
}

TEST(ShotThreat, AdversarioAlinhadoComOGolEPertoDaBola)
{
  // Nosso gol em x = -2200. Bola em (-1500, 0); adversário atrás dela (x maior), apontando para o gol (180 graus).
  const gk::Vec2 gol{-2200.0, 0.0};
  const gk::Vec2 bola{-1500.0, 0.0};
  EXPECT_TRUE(gk::isShotThreat({-1200.0, 0.0}, M_PI, bola, gol));
  // Mesma posição, mas olhando para o outro lado: sem ameaça.
  EXPECT_FALSE(gk::isShotThreat({-1200.0, 0.0}, 0.0, bola, gol));
  // Longe da bola: sem ameaça.
  EXPECT_FALSE(gk::isShotThreat({0.0, 0.0}, M_PI, bola, gol));
  // Do lado do gol em relação à bola (a bola não está entre ele e o gol): sem ameaça.
  EXPECT_FALSE(gk::isShotThreat({-1800.0, 0.0}, M_PI, bola, gol));
}

TEST(ApproachBlocked, AdversarioNoCaminhoBloqueiaMasNaBolaNao)
{
  const gk::Vec2 goleiro{-2000.0, 0.0};
  const gk::Vec2 bola{-1000.0, 0.0};
  // Adversário no meio do caminho
  EXPECT_TRUE(gk::approachBlocked(goleiro, bola, {{-1500.0, 100.0}}));
  // Adversário colado na bola (último trecho): não conta
  EXPECT_FALSE(gk::approachBlocked(goleiro, bola, {{-1100.0, 100.0}}));
  // Adversário longe do caminho
  EXPECT_FALSE(gk::approachBlocked(goleiro, bola, {{-1500.0, 800.0}}));
  // Bola já perto do goleiro: nada a bloquear
  EXPECT_FALSE(gk::approachBlocked({-1100.0, 0.0}, bola, {{-1050.0, 0.0}}));
}

TEST(ChooseClearAngleWithPossession, EstabilidadeEvitaTrocarDeLadoComEscoresProximos)
{
  // Aliado e adversário simétricos em relação à bola: duas direções (para cima e para baixo) empatam.
  const gk::Vec2 ball{-1500.0, 0.0};
  const std::vector<gk::Vec2> aliados{{-300.0, 800.0}, {-300.0, -800.0}};
  const auto livre = gk::chooseClearAngleWithPossession(
    ball, 0.0, 3.0, false, aliados, {}, 2200.0, 1500.0);
  const double anterior = livre.angle + 10.0 * M_PI / 180.0;   // a direção vizinha, quase tão boa quanto
  const auto estavel = gk::chooseClearAngleWithPossession(
    ball, 0.0, 3.0, false, aliados, {}, 2200.0, 1500.0, gk::PassParams(), gk::FanParams(),
    gk::DestinationParams(), 400.0, 1.0, anterior, 0.5);
  EXPECT_NEAR(estavel.angle, anterior, 1e-6);
}
