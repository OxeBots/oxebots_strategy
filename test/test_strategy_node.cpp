// Copyright 2024 Oxebots
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#define STRATEGY_NODE_NO_MAIN
#include "../src/strategy_node.cpp"

using GameState = oxebots_interfaces::msg::GameState;
using GameData = oxebots_interfaces::msg::GameData;
using SSLGeometryData = oxebots_interfaces::msg::SSLGeometryData;
using RoleAssignment = oxebots_interfaces::msg::RoleAssignment;

class StrategyNodeBlackboardTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static SSLGeometryData::SharedPtr makeGeometry(double length = 4500.0, double width = 3000.0)
    {
        auto msg = std::make_shared<SSLGeometryData>();
        msg->field.field_length = static_cast<float>(length);
        msg->field.field_width = static_cast<float>(width);
        msg->field.penalty_area_depth = 800.0f;
        msg->field.penalty_area_width = 1600.0f;
        msg->field.goal_width = 1000.0f;
        msg->field.center_circle_radius = 500.0f;
        return msg;
    }

    static GameState::SharedPtr makeGameState(bool our_goal_positive_x, uint8_t phase = GameState::PHASE_RUN, bool we_favored = true)
    {
        auto msg = std::make_shared<GameState>();
        msg->header.stamp = rclcpp::Clock().now();
        msg->our_goal_positive_x = our_goal_positive_x;
        msg->phase = phase;
        msg->command = 2; // NORMAL_START
        msg->stage = 0;
        msg->we_favored = we_favored;
        msg->speed_limit_mps = 1.5f;
        msg->can_place_ball = false;
        msg->has_designated = false;
        return msg;
    }
};

// ============================================================================
// 1. Inicialização da Blackboard (D03 §2) e desacoplamento de is_yellow
// ============================================================================

TEST_F(StrategyNodeBlackboardTest, SetupBlackboardInitializesKeysAndKeepsTrueColor)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("is_yellow_team", true);
    opts.append_parameter_override("invert_sides", true); // Invert sides não deve inverter is_yellow na blackboard
    opts.append_parameter_override("robot_id", 1);

    auto node = std::make_shared<StrategyNode>(opts);
    auto bb = node->getBlackboard();

    // D03: is_yellow volta a significar exclusivamente a cor real do time
    bool is_yellow = false;
    ASSERT_TRUE(bb->get("is_yellow", is_yellow));
    EXPECT_TRUE(is_yellow);

    // Chaves de geometria e lado inicializadas
    double own_goal_x = 0.0, opp_goal_x = 0.0, my_goal_x = 0.0;
    int side_sign = 0;
    EXPECT_TRUE(bb->get("own_goal_x", own_goal_x));
    EXPECT_TRUE(bb->get("opp_goal_x", opp_goal_x));
    EXPECT_TRUE(bb->get("my_goal_x", my_goal_x));
    EXPECT_TRUE(bb->get("side_sign", side_sign));
    EXPECT_EQ(side_sign, 1);

    // Chaves de estado e idades
    int phase = -1;
    double speed_limit = 0.0;
    double gc_age_ms = 0.0, pose_age_ms = 0.0, ball_age_ms = 0.0;
    EXPECT_TRUE(bb->get("phase", phase));
    EXPECT_TRUE(bb->get("speed_limit", speed_limit));
    EXPECT_TRUE(bb->get("gc_age_ms", gc_age_ms));
    EXPECT_TRUE(bb->get("pose_age_ms", pose_age_ms));
    EXPECT_TRUE(bb->get("ball_age_ms", ball_age_ms));
    EXPECT_TRUE(std::isinf(gc_age_ms));
    EXPECT_TRUE(std::isinf(pose_age_ms));
    EXPECT_TRUE(std::isinf(ball_age_ms));
}

// ============================================================================
// 2. Teste de Aceite SC-22: Troca de lado no 2º tempo (blue_team_on_positive_half)
// ============================================================================

TEST_F(StrategyNodeBlackboardTest, SC22_SideSwitchBetweenHalves)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("is_yellow_team", false); // Time azul
    opts.append_parameter_override("robot_id", 1);

    auto node = std::make_shared<StrategyNode>(opts);
    auto bb = node->getBlackboard();

    const double field_len = 4500.0;
    const double half_len = field_len / 2.0;
    node->setLastGeometry(makeGeometry(field_len, 3000.0));

    // 1º TEMPO: blue_team_on_positive_half = false -> our_goal_positive_x = false
    // Time azul defende o gol de X negativo
    node->setLastGameState(makeGameState(/*our_goal_positive_x=*/false));
    node->refresh();

    double own_goal_x = 0.0, opp_goal_x = 0.0, my_goal_x = 0.0;
    int side_sign = 0;
    ASSERT_TRUE(bb->get("own_goal_x", own_goal_x));
    ASSERT_TRUE(bb->get("opp_goal_x", opp_goal_x));
    ASSERT_TRUE(bb->get("my_goal_x", my_goal_x));
    ASSERT_TRUE(bb->get("side_sign", side_sign));

    EXPECT_DOUBLE_EQ(own_goal_x, -half_len);
    EXPECT_DOUBLE_EQ(opp_goal_x, half_len);
    EXPECT_DOUBLE_EQ(my_goal_x, -half_len);
    EXPECT_EQ(side_sign, 1); // +1 se defendemos x < 0

    bool is_yellow = true;
    ASSERT_TRUE(bb->get("is_yellow", is_yellow));
    EXPECT_FALSE(is_yellow); // Permanece azul

    // 2º TEMPO: blue_team_on_positive_half = true -> our_goal_positive_x = true
    // Times invertem de lado; time azul agora defende o gol de X positivo
    node->setLastGameState(makeGameState(/*our_goal_positive_x=*/true));
    node->refresh();

    ASSERT_TRUE(bb->get("own_goal_x", own_goal_x));
    ASSERT_TRUE(bb->get("opp_goal_x", opp_goal_x));
    ASSERT_TRUE(bb->get("my_goal_x", my_goal_x));
    ASSERT_TRUE(bb->get("side_sign", side_sign));

    EXPECT_DOUBLE_EQ(own_goal_x, half_len);
    EXPECT_DOUBLE_EQ(opp_goal_x, -half_len);
    EXPECT_DOUBLE_EQ(my_goal_x, half_len);
    EXPECT_EQ(side_sign, -1); // -1 se defendemos x > 0

    // Cor do time NÃO é afetada pela troca de lado
    ASSERT_TRUE(bb->get("is_yellow", is_yellow));
    EXPECT_FALSE(is_yellow);
}

// ============================================================================
// 3. Atualização de Poses, Velocidades e Cálculo de Idades (refresh)
// ============================================================================

TEST_F(StrategyNodeBlackboardTest, RefreshComputesStampsAndPoses)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("is_yellow_team", false);
    opts.append_parameter_override("robot_id", 1);

    auto node = std::make_shared<StrategyNode>(opts);
    auto bb = node->getBlackboard();

    node->setLastGeometry(makeGeometry(4500.0, 3000.0));
    node->setLastGameState(makeGameState(false));

    auto gd = std::make_shared<GameData>();
    gd->ball.x = 200.0f;
    gd->ball.y = -100.0f;

    oxebots_interfaces::msg::RobotGameData ally;
    ally.id = 1;
    ally.x = -500.0f;
    ally.y = 150.0f;
    ally.orientation = 0.5f;
    gd->robots.allies.push_back(ally);

    node->setLastGameData(gd);
    node->refresh();

    Pose2D robot_pose{0.0, 0.0, 0.0};
    Pose2D ball_pose{0.0, 0.0, 0.0};
    double pose_age_ms = -1.0;
    double ball_age_ms = -1.0;

    ASSERT_TRUE(bb->get("robot_pose", robot_pose));
    EXPECT_NEAR(robot_pose.x, -500.0, 1e-3);
    EXPECT_NEAR(robot_pose.y, 150.0, 1e-3);
    EXPECT_NEAR(robot_pose.theta, 0.5, 1e-3);

    ASSERT_TRUE(bb->get("ball", ball_pose));
    EXPECT_NEAR(ball_pose.x, 200.0, 1e-3);
    EXPECT_NEAR(ball_pose.y, -100.0, 1e-3);

    ASSERT_TRUE(bb->get("pose_age_ms", pose_age_ms));
    ASSERT_TRUE(bb->get("ball_age_ms", ball_age_ms));
    EXPECT_GE(pose_age_ms, 0.0);
    EXPECT_LT(pose_age_ms, 1000.0);
    EXPECT_GE(ball_age_ms, 0.0);
    EXPECT_LT(ball_age_ms, 1000.0);
}

// ============================================================================
// 4. Consumo da RoleAssignment ampliada na Blackboard (P02)
// ============================================================================

TEST_F(StrategyNodeBlackboardTest, P02_RoleAssignmentUpdatesEpochKickerAndActiveIds)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("is_yellow_team", false);
    opts.append_parameter_override("robot_id", 2);

    auto node = std::make_shared<StrategyNode>(opts);
    auto bb = node->getBlackboard();

    // Antes de receber RoleAssignment, o papel deve iniciar como "idle" (sem papéis fixos chumbados)
    std::string initial_role;
    uint32_t initial_epoch = 999;
    ASSERT_TRUE(bb->get("role", initial_role));
    ASSERT_TRUE(bb->get("role_epoch", initial_epoch));
    EXPECT_EQ(initial_role, "idle");
    EXPECT_EQ(initial_epoch, 0u);

    // Publica um RoleAssignment com robô 2 como atacante e cobrador (epoch = 3)
    auto roles = std::make_shared<RoleAssignment>();
    roles->goalkeeper_id = 0;
    roles->attacker_id = 2;
    roles->defender_id = 1;
    roles->kicker_id = 2;
    roles->epoch = 3;
    roles->active_ids = {0, 1, 2};

    node->setLastRoleAssignment(roles);
    node->refresh();

    std::string role;
    uint32_t gk_id = 999, att_id = 999, def_id = 999, kick_id = 999, epoch = 0;
    bool is_gk = true, is_att = false, is_def = true, is_kick = false;
    std::vector<uint32_t> active_ids;

    ASSERT_TRUE(bb->get("role", role));
    ASSERT_TRUE(bb->get("goalkeeper_id", gk_id));
    ASSERT_TRUE(bb->get("attacker_id", att_id));
    ASSERT_TRUE(bb->get("defender_id", def_id));
    ASSERT_TRUE(bb->get("kicker_id", kick_id));
    ASSERT_TRUE(bb->get("role_epoch", epoch));
    ASSERT_TRUE(bb->get("active_ids", active_ids));
    ASSERT_TRUE(bb->get("is_goalkeeper", is_gk));
    ASSERT_TRUE(bb->get("is_attacker", is_att));
    ASSERT_TRUE(bb->get("is_defender", is_def));
    ASSERT_TRUE(bb->get("is_kicker", is_kick));

    EXPECT_EQ(role, "attacker");
    EXPECT_EQ(gk_id, 0u);
    EXPECT_EQ(att_id, 2u);
    EXPECT_EQ(def_id, 1u);
    EXPECT_EQ(kick_id, 2u);
    EXPECT_EQ(epoch, 3u);
    ASSERT_EQ(active_ids.size(), 3u);
    EXPECT_FALSE(is_gk);
    EXPECT_TRUE(is_att);
    EXPECT_FALSE(is_def);
    EXPECT_TRUE(is_kick);
}
