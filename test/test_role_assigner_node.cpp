// Copyright 2024 Oxebots
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#define ROLE_ASSIGNER_NODE_NO_MAIN
#include "../src/role_assigner_node.cpp"

using GameData = oxebots_interfaces::msg::GameData;
using GameState = oxebots_interfaces::msg::GameState;
using RoleAssignment = oxebots_interfaces::msg::RoleAssignment;
using RobotGameData = oxebots_interfaces::msg::RobotGameData;

class RoleAssignerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static RobotGameData makeRobot(uint32_t id, float x, float y, float ori = 0.0f)
  {
    RobotGameData r;
    r.id = id;
    r.x = x;
    r.y = y;
    r.orientation = ori;
    return r;
  }

  // max_bots = 0: o GC não informou o limite de robôs (sem retirada).
  static GameState::SharedPtr makeGameState(
    uint8_t phase, uint32_t gk_id = 0, bool we_favored = true, bool stale = false,
    uint8_t max_bots = 0)
  {
    auto gs = std::make_shared<GameState>();
    gs->phase = phase;
    gs->our_goalkeeper_id = gk_id;
    gs->we_favored = we_favored;
    gs->stale = stale;
    gs->max_allowed_bots = max_bots;
    return gs;
  }

  static GameData::SharedPtr makeThreeRobots()
  {
    auto gd = std::make_shared<GameData>();
    gd->ball.x = 0.0f;
    gd->ball.y = 0.0f;
    gd->robots.allies = {
      makeRobot(0, -2000.0f, 0.0f),  // Goleiro
      makeRobot(1, -200.0f, 0.0f),   // Perto da bola
      makeRobot(2, -1500.0f, 0.0f)   // Longe da bola
    };
    return gd;
  }
};

// ============================================================================
// 1. SC-18: Exclusividade de papéis, histerese e incremento de epoch em RUN
// ============================================================================
TEST_F(RoleAssignerNodeTest, SC18_ExclusivityHysteresisAndEpochInRun)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("time_hysteresis", 0.05); // 50 ms para teste rápido
  auto node = std::make_shared<RoleAssignerNode>(opts);

  node->processGameState(makeGameState(GameState::PHASE_RUN, 0));

  auto gd = std::make_shared<GameData>();
  gd->ball.x = 0.0f;
  gd->ball.y = 0.0f;
  gd->robots.allies = {
    makeRobot(0, -2000.0f, 0.0f), // Goleiro
    makeRobot(1, -200.0f, 0.0f),  // Muito perto da bola -> Atacante
    makeRobot(2, -1500.0f, 0.0f)  // Longe da bola -> Defensor
  };

  node->processGameData(gd);
  auto roles1 = node->getLastPublishedRoles();

  EXPECT_EQ(roles1.goalkeeper_id, 0u);
  EXPECT_EQ(roles1.attacker_id, 1u);
  EXPECT_EQ(roles1.defender_id, 2u);
  EXPECT_EQ(roles1.kicker_id, RoleAssignment::NO_ROBOT);
  EXPECT_EQ(roles1.epoch, 1u);
  ASSERT_EQ(roles1.active_ids.size(), 3u);

  // Republicar os mesmos dados NÃO deve incrementar a epoch
  node->processGameData(gd);
  auto roles_same = node->getLastPublishedRoles();
  EXPECT_EQ(roles_same.epoch, 1u);

  // Robô 2 passa só um pouco mais perto (mas dentro da histerese de custo 2.5): robô 1 mantém papel
  gd->robots.allies[1] = makeRobot(1, -300.0f, 0.0f);
  gd->robots.allies[2] = makeRobot(2, -250.0f, 0.0f);
  node->processGameData(gd);
  EXPECT_EQ(node->getLastPublishedRoles().attacker_id, 1u);
  EXPECT_EQ(node->getLastPublishedRoles().epoch, 1u);
}

// ============================================================================
// 2. P01: Cobrador travado durante fases de reinício (PREPARE_KICKOFF -> RUN)
// ============================================================================
TEST_F(RoleAssignerNodeTest, P01_LockedKickerDuringRestartPhases)
{
  auto node = std::make_shared<RoleAssignerNode>();

  // Entra em PREPARE_KICKOFF com robô 1 mais perto da bola
  node->processGameState(makeGameState(GameState::PHASE_PREPARE_KICKOFF, 0));

  auto gd = std::make_shared<GameData>();
  gd->ball.x = 0.0f;
  gd->ball.y = 0.0f;
  gd->robots.allies = {
    makeRobot(0, -2000.0f, 0.0f),
    makeRobot(1, -300.0f, 0.0f),
    makeRobot(2, -1500.0f, 0.0f)
  };
  node->processGameData(gd);

  auto roles_prep = node->getLastPublishedRoles();
  EXPECT_EQ(roles_prep.kicker_id, 1u);
  EXPECT_EQ(roles_prep.attacker_id, 1u);
  EXPECT_EQ(roles_prep.defender_id, 2u);

  // Mesmo que o robô 2 passe a ficar colado na bola e o robô 1 longe durante KICKOFF,
  // o cobrador (kicker_id = 1) permanece TRAVADO!
  node->processGameState(makeGameState(GameState::PHASE_KICKOFF, 0));
  gd->robots.allies[1] = makeRobot(1, -1800.0f, 0.0f);
  gd->robots.allies[2] = makeRobot(2, -50.0f, 0.0f);
  node->processGameData(gd);

  auto roles_kick = node->getLastPublishedRoles();
  EXPECT_EQ(roles_kick.kicker_id, 1u);
  EXPECT_EQ(roles_kick.attacker_id, 1u);
  EXPECT_EQ(roles_kick.defender_id, 2u);

  // Ao voltar para PHASE_RUN, o kicker_id é liberado (NO_ROBOT)
  node->processGameState(makeGameState(GameState::PHASE_RUN, 0));
  node->processGameData(gd);
  EXPECT_EQ(node->getLastPublishedRoles().kicker_id, RoleAssignment::NO_ROBOT);
}

// ============================================================================
// 3. P01: Goleiro dinâmico do GC e exclusividade com 1 único robô de linha
// ============================================================================
TEST_F(RoleAssignerNodeTest, P01_DynamicGoalkeeperFromGCAndSingleFieldRobot)
{
  auto node = std::make_shared<RoleAssignerNode>();

  // GC define robô 2 como goleiro; apenas robôs 2 e 0 estão em campo
  node->processGameState(makeGameState(GameState::PHASE_RUN, 2));

  auto gd = std::make_shared<GameData>();
  gd->ball.x = 0.0f;
  gd->ball.y = 0.0f;
  gd->robots.allies = {
    makeRobot(0, -400.0f, 0.0f),
    makeRobot(2, -2000.0f, 0.0f)
  };
  node->processGameData(gd);

  auto roles = node->getLastPublishedRoles();
  EXPECT_EQ(roles.goalkeeper_id, 2u);
  EXPECT_EQ(roles.attacker_id, 0u);
  EXPECT_EQ(roles.defender_id, RoleAssignment::NO_ROBOT);
}

// ============================================================================
// 4. Revisão PR: Reinício do oponente (we_favored=false), fallback stale e ordenação de active_ids
// ============================================================================
TEST_F(RoleAssignerNodeTest, P01_OpponentRestartNoKickerStaleFallbackAndSortedActiveIds)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("goalkeeper_id", 1);
  auto node = std::make_shared<RoleAssignerNode>(opts);

  // 1) Quando GC está stale=true, deve fazer fallback para o parâmetro goalkeeper_id (1), ignorando our_goalkeeper_id (0)
  node->processGameState(makeGameState(GameState::PHASE_FREE_KICK, /*gk_id=*/0, /*we_favored=*/false, /*stale=*/true));

  auto gd = std::make_shared<GameData>();
  gd->ball.x = 0.0f;
  gd->ball.y = 0.0f;
  // Ordem invertida (2, 0, 1) para testar ordenação defensiva de active_ids
  gd->robots.allies = {
    makeRobot(2, -1500.0f, 0.0f),
    makeRobot(0, -300.0f, 0.0f),
    makeRobot(1, -2000.0f, 0.0f)
  };
  node->processGameData(gd);

  auto roles1 = node->getLastPublishedRoles();
  EXPECT_EQ(roles1.goalkeeper_id, 1u);
  // 2) Como we_favored == false (falta do oponente), kicker_id deve ser NO_ROBOT
  EXPECT_EQ(roles1.kicker_id, RoleAssignment::NO_ROBOT);
  ASSERT_EQ(roles1.active_ids.size(), 3u);
  EXPECT_EQ(roles1.active_ids[0], 0u);
  EXPECT_EQ(roles1.active_ids[1], 1u);
  EXPECT_EQ(roles1.active_ids[2], 2u);
  EXPECT_EQ(roles1.epoch, 1u);

  // 3) Mudar a ordem dos robôs em msg->robots.allies NÃO deve incrementar a epoch
  gd->robots.allies = {
    makeRobot(1, -2000.0f, 0.0f),
    makeRobot(0, -300.0f, 0.0f),
    makeRobot(2, -1500.0f, 0.0f)
  };
  node->processGameData(gd);
  EXPECT_EQ(node->getLastPublishedRoles().epoch, 1u);
}

// ============================================================================
// 5. P03 / SC-23: cartão amarelo (max_allowed_bots 3 -> 2) e robô perdido
// ============================================================================
TEST_F(RoleAssignerNodeTest, SC23_YellowCardKeepsGoalkeeperAndBestLineRobot)
{
  auto node = std::make_shared<RoleAssignerNode>();
  auto gd = makeThreeRobots();

  node->processGameState(makeGameState(GameState::PHASE_RUN, 0, true, false, /*max_bots=*/3));
  node->processGameData(gd);
  auto full = node->getLastPublishedRoles();
  EXPECT_EQ(full.goalkeeper_id, 0u);
  EXPECT_EQ(full.attacker_id, 1u);
  EXPECT_EQ(full.defender_id, 2u);
  ASSERT_EQ(full.active_ids.size(), 3u);
  const uint32_t epoch_full = full.epoch;

  // Cartão amarelo: o GC passa o limite para 2 com os 3 robôs ainda em campo.
  node->processGameState(makeGameState(GameState::PHASE_RUN, 0, true, false, /*max_bots=*/2));
  node->processGameData(gd);
  auto reduced = node->getLastPublishedRoles();
  EXPECT_EQ(reduced.goalkeeper_id, 0u);
  EXPECT_EQ(reduced.attacker_id, 1u);
  EXPECT_EQ(reduced.defender_id, RoleAssignment::NO_ROBOT);
  EXPECT_EQ(reduced.kicker_id, RoleAssignment::NO_ROBOT);
  ASSERT_EQ(reduced.active_ids.size(), 2u);
  EXPECT_EQ(reduced.active_ids[0], 0u);
  EXPECT_EQ(reduced.active_ids[1], 1u);  // o robô 2 (pior custo) saiu de active_ids
  EXPECT_GT(reduced.epoch, epoch_full);

  // O robô retirado fica retirado mesmo que passe a ter custo melhor: a escolha é pegajosa, para o
  // humano não ver a retirada trocar de robô enquanto o tira do campo.
  gd->robots.allies[1] = makeRobot(1, -1600.0f, 0.0f);
  gd->robots.allies[2] = makeRobot(2, -100.0f, 0.0f);
  node->processGameData(gd);
  auto sticky = node->getLastPublishedRoles();
  EXPECT_EQ(sticky.attacker_id, 1u);
  EXPECT_EQ(sticky.defender_id, RoleAssignment::NO_ROBOT);
  ASSERT_EQ(sticky.active_ids.size(), 2u);
  EXPECT_EQ(sticky.active_ids[1], 1u);

  // O cartão expira: o limite volta a 3 e os três robôs voltam a ter papel.
  node->processGameState(makeGameState(GameState::PHASE_RUN, 0, true, false, /*max_bots=*/3));
  node->processGameData(gd);
  auto restored = node->getLastPublishedRoles();
  ASSERT_EQ(restored.active_ids.size(), 3u);
  EXPECT_NE(restored.attacker_id, RoleAssignment::NO_ROBOT);
  EXPECT_NE(restored.defender_id, RoleAssignment::NO_ROBOT);
  EXPECT_NE(restored.attacker_id, restored.defender_id);
}

TEST_F(RoleAssignerNodeTest, P03_TwoLimitCardsLeaveOnlyTheGoalkeeper)
{
  auto node = std::make_shared<RoleAssignerNode>();
  node->processGameState(makeGameState(GameState::PHASE_RUN, 0, true, false, /*max_bots=*/1));
  node->processGameData(makeThreeRobots());

  auto roles = node->getLastPublishedRoles();
  EXPECT_EQ(roles.goalkeeper_id, 0u);
  EXPECT_EQ(roles.attacker_id, RoleAssignment::NO_ROBOT);
  EXPECT_EQ(roles.defender_id, RoleAssignment::NO_ROBOT);
  ASSERT_EQ(roles.active_ids.size(), 1u);
  EXPECT_EQ(roles.active_ids[0], 0u);
}

TEST_F(RoleAssignerNodeTest, P03_UnknownLimitDoesNotRetireAnyone)
{
  // max_allowed_bots = 0 significa "o GC não informou": não retira ninguém.
  auto node = std::make_shared<RoleAssignerNode>();
  node->processGameState(makeGameState(GameState::PHASE_RUN, 0, true, false, /*max_bots=*/0));
  node->processGameData(makeThreeRobots());

  auto roles = node->getLastPublishedRoles();
  EXPECT_EQ(roles.attacker_id, 1u);
  EXPECT_EQ(roles.defender_id, 2u);
  EXPECT_EQ(roles.active_ids.size(), 3u);
}

TEST_F(RoleAssignerNodeTest, P03_LimitAtOrAboveRobotsInFieldDoesNotRetireAnyone)
{
  auto node = std::make_shared<RoleAssignerNode>();
  node->processGameState(makeGameState(GameState::PHASE_RUN, 0, true, false, /*max_bots=*/8));
  node->processGameData(makeThreeRobots());
  EXPECT_EQ(node->getLastPublishedRoles().active_ids.size(), 3u);
}

TEST_F(RoleAssignerNodeTest, P03_RemovedRobotLeavingVisionMeetsTheLimitWithoutRetiringAnotherOne)
{
  auto node = std::make_shared<RoleAssignerNode>();
  auto gd = makeThreeRobots();
  node->processGameState(makeGameState(GameState::PHASE_RUN, 0, true, false, /*max_bots=*/2));
  node->processGameData(gd);
  ASSERT_EQ(node->getLastPublishedRoles().active_ids.size(), 2u);  // robô 2 retirado

  // O humano tira o robô 2: ele some da visão. O limite agora é cumprido sem retirar mais ninguém.
  gd->robots.allies.pop_back();
  node->processGameData(gd);
  auto roles = node->getLastPublishedRoles();
  EXPECT_EQ(roles.goalkeeper_id, 0u);
  EXPECT_EQ(roles.attacker_id, 1u);
  EXPECT_EQ(roles.defender_id, RoleAssignment::NO_ROBOT);
  ASSERT_EQ(roles.active_ids.size(), 2u);
}

TEST_F(RoleAssignerNodeTest, P03_LostRobotRebuildsTheRolesAndTheTeamRecovers)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("time_hysteresis", 0.0);
  auto node = std::make_shared<RoleAssignerNode>(opts);
  auto gd = makeThreeRobots();
  node->processGameState(makeGameState(GameState::PHASE_RUN, 0, true, false, /*max_bots=*/3));
  node->processGameData(gd);
  ASSERT_EQ(node->getLastPublishedRoles().attacker_id, 1u);

  // O robô 1 (atacante) some da visão: o robô 2 assume o ataque e não há defensor.
  auto lost = std::make_shared<GameData>(*gd);
  lost->robots.allies.erase(lost->robots.allies.begin() + 1);
  node->processGameData(lost);
  auto degraded = node->getLastPublishedRoles();
  EXPECT_EQ(degraded.goalkeeper_id, 0u);
  EXPECT_EQ(degraded.attacker_id, 2u);
  EXPECT_EQ(degraded.defender_id, RoleAssignment::NO_ROBOT);
  ASSERT_EQ(degraded.active_ids.size(), 2u);

  // Ele volta à visão: os dois robôs de linha voltam a ter papéis exclusivos.
  node->processGameData(gd);
  auto recovered = node->getLastPublishedRoles();
  ASSERT_EQ(recovered.active_ids.size(), 3u);
  EXPECT_NE(recovered.attacker_id, RoleAssignment::NO_ROBOT);
  EXPECT_NE(recovered.defender_id, RoleAssignment::NO_ROBOT);
  EXPECT_NE(recovered.attacker_id, recovered.defender_id);
}

TEST_F(RoleAssignerNodeTest, P03_WithoutGoalkeeperTheTwoLineRobotsAreAttackerAndDefender)
{
  auto node = std::make_shared<RoleAssignerNode>();
  node->processGameState(makeGameState(GameState::PHASE_RUN, 0, true, false, /*max_bots=*/3));

  auto gd = std::make_shared<GameData>();
  gd->ball.x = 0.0f;
  gd->ball.y = 0.0f;
  gd->robots.allies = {makeRobot(1, -200.0f, 0.0f), makeRobot(2, -1500.0f, 0.0f)};  // sem o robô 0
  node->processGameData(gd);

  auto roles = node->getLastPublishedRoles();
  EXPECT_EQ(roles.goalkeeper_id, RoleAssignment::NO_ROBOT);
  EXPECT_EQ(roles.attacker_id, 1u);
  EXPECT_EQ(roles.defender_id, 2u);
  EXPECT_EQ(roles.active_ids.size(), 2u);
}

TEST_F(RoleAssignerNodeTest, P03_CardDuringOurKickoffRelocksTheKickerWhenTheLockedOneIsRetired)
{
  auto node = std::make_shared<RoleAssignerNode>();
  auto gd = makeThreeRobots();
  node->processGameState(
    makeGameState(GameState::PHASE_PREPARE_KICKOFF, 0, true, false, /*max_bots=*/3));
  node->processGameData(gd);
  ASSERT_EQ(node->getLastPublishedRoles().kicker_id, 1u);

  // O cobrador travado (robô 1) fica o pior custo e o cartão tira um robô: ele é o retirado, e o
  // cobrador passa para quem ficou.
  gd->robots.allies[1] = makeRobot(1, -1900.0f, 800.0f);
  gd->robots.allies[2] = makeRobot(2, -100.0f, 0.0f);
  node->processGameState(
    makeGameState(GameState::PHASE_PREPARE_KICKOFF, 0, true, false, /*max_bots=*/2));
  node->processGameData(gd);

  auto roles = node->getLastPublishedRoles();
  EXPECT_EQ(roles.goalkeeper_id, 0u);
  EXPECT_EQ(roles.kicker_id, 2u);
  EXPECT_EQ(roles.attacker_id, 2u);
  EXPECT_EQ(roles.defender_id, RoleAssignment::NO_ROBOT);
  ASSERT_EQ(roles.active_ids.size(), 2u);
}

// ============================================================================
// 6. Toque na bola troca papel de atacante imediatamente (ignora histereses)
// ============================================================================
TEST_F(RoleAssignerNodeTest, ImmediateAttackerSwitchWhenBallTouchesPlayer)
{
  auto node = std::make_shared<RoleAssignerNode>();
  node->processGameState(makeGameState(GameState::PHASE_RUN, 0));

  auto gd = std::make_shared<GameData>();
  gd->ball.x = 0.0f;
  gd->ball.y = 0.0f;
  gd->robots.allies = {
    makeRobot(0, -2000.0f, 0.0f),
    makeRobot(1, -200.0f, 0.0f),   // Atacante inicial
    makeRobot(2, -1000.0f, 0.0f)   // Defensor inicial
  };
  node->processGameData(gd);
  ASSERT_EQ(node->getLastPublishedRoles().attacker_id, 1u);
  ASSERT_EQ(node->getLastPublishedRoles().defender_id, 2u);

  // A bola bate/encosta no robô 2 (dist = 110 mm <= 160 mm) no mesmo instante (dentro de
  // time_hysteresis = 2.0 s e com robô 1 ainda a 690 mm dentro do bônus de histerese 2.5):
  // robô 2 deve virar atacante NA MESMA HORA.
  gd->ball.x = -890.0f;
  gd->ball.y = 0.0f;
  node->processGameData(gd);

  auto roles = node->getLastPublishedRoles();
  EXPECT_EQ(roles.attacker_id, 2u);
  EXPECT_EQ(roles.defender_id, 1u);

  // Quando o novo atacante (robô 2) recua até o ponto de pré-chute (~400 mm atrás da bola),
  // o papel NÃO deve oscilar de volta para o robô 1.
  gd->robots.allies[2] = makeRobot(2, -1290.0f, 0.0f);  // dist = 400 mm da bola em (-890, 0)
  node->processGameData(gd);
  EXPECT_EQ(node->getLastPublishedRoles().attacker_id, 2u);
  EXPECT_EQ(node->getLastPublishedRoles().defender_id, 1u);
}

// ============================================================================
// 7. Pós-lateral / escanteio (FREE_KICK): quem não bateu vira atacante assim que bater
// ============================================================================
TEST_F(RoleAssignerNodeTest, ThrowInAndCornerKickHandoverToNonKickerImmediatelyAfterKick)
{
  auto node = std::make_shared<RoleAssignerNode>();
  node->processGameState(makeGameState(GameState::PHASE_FREE_KICK, 0, /*we_favored=*/true));

  auto gd = std::make_shared<GameData>();
  gd->ball.x = 0.0f;
  gd->ball.y = 1300.0f;
  gd->robots.allies = {
    makeRobot(0, -2000.0f, 0.0f),   // Goleiro
    makeRobot(1, 0.0f, 1420.0f),    // Cobrador do lateral (dist = 120 mm)
    makeRobot(2, 600.0f, 300.0f)    // Recebedor posicionado no meio (dist ~ 1166 mm)
  };
  node->processGameData(gd);

  auto roles_fk = node->getLastPublishedRoles();
  ASSERT_EQ(roles_fk.kicker_id, 1u);
  ASSERT_EQ(roles_fk.attacker_id, 1u);
  ASSERT_EQ(roles_fk.defender_id, 2u);

  // Assim que o robô 1 bate o lateral (bola se desloca >= 50 mm e/ou fase muda para RUN),
  // mesmo com a bola ainda colada no robô 1 (dist ~ 180 mm) e longe do robô 2,
  // o robô 2 (que não bateu) vira atacante na mesma hora para evitar recuo maluco.
  gd->ball.x = 30.0f;
  gd->ball.y = 1240.0f;
  node->processGameState(makeGameState(GameState::PHASE_RUN, 0));
  node->processGameData(gd);

  auto roles_run = node->getLastPublishedRoles();
  EXPECT_EQ(roles_run.kicker_id, RoleAssignment::NO_ROBOT);
  EXPECT_EQ(roles_run.attacker_id, 2u);
  EXPECT_EQ(roles_run.defender_id, 1u);
}

// ============================================================================
// 12. Priorização de robô com kicker operacional como cobrador de bola parada
// ============================================================================
TEST_F(RoleAssignerNodeTest, OperationalKickersPrioritizesLineRobot)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("operational_kickers", std::vector<int64_t>{0, 2});
  auto node = std::make_shared<RoleAssignerNode>(opts);

  // Bola parada (PREPARE_KICKOFF).
  // Robô 1 está mais perto da bola (-300 mm), mas kicker quebrado (não listado).
  // Robô 2 está mais longe (-1500 mm), mas kicker operacional.
  // Goleiro (0) está listado, mas NUNCA deve ser cobrador.
  node->processGameState(makeGameState(GameState::PHASE_PREPARE_KICKOFF, 0));

  auto gd = std::make_shared<GameData>();
  gd->ball.x = 0.0f;
  gd->ball.y = 0.0f;
  gd->robots.allies = {
    makeRobot(0, -2000.0f, 0.0f),
    makeRobot(1, -300.0f, 0.0f),
    makeRobot(2, -1500.0f, 0.0f)
  };
  node->processGameData(gd);

  auto roles = node->getLastPublishedRoles();
  EXPECT_EQ(roles.kicker_id, 2u);
  EXPECT_EQ(roles.attacker_id, 2u);
  EXPECT_EQ(roles.defender_id, 1u);
  EXPECT_EQ(roles.goalkeeper_id, 0u);
}

// ============================================================================
// 13. Goleiro solo cobra bola parada quando não há robô de linha em campo
// ============================================================================
TEST_F(RoleAssignerNodeTest, SoloGoalkeeperBecomesKickerOnOurSetPieces)
{
  auto node = std::make_shared<RoleAssignerNode>();
  // Apenas 1 robô permitido (ou apenas o goleiro 0 visível em campo) durante nosso kickoff
  node->processGameState(makeGameState(GameState::PHASE_PREPARE_KICKOFF, 0, true, false, /*max_bots=*/1));
  node->processGameData(makeThreeRobots());

  auto roles = node->getLastPublishedRoles();
  EXPECT_EQ(roles.goalkeeper_id, 0u);
  EXPECT_EQ(roles.kicker_id, 0u);
  EXPECT_EQ(roles.attacker_id, RoleAssignment::NO_ROBOT);
  EXPECT_EQ(roles.defender_id, RoleAssignment::NO_ROBOT);
  ASSERT_EQ(roles.active_ids.size(), 1u);
  EXPECT_EQ(roles.active_ids[0], 0u);
}
