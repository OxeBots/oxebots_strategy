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

  static GameState::SharedPtr makeGameState(uint8_t phase, uint32_t gk_id = 0)
  {
    auto gs = std::make_shared<GameState>();
    gs->phase = phase;
    gs->our_goalkeeper_id = gk_id;
    return gs;
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
