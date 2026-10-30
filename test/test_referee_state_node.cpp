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

#define REFEREE_STATE_NODE_NO_MAIN
#include "../src/referee_state_node.cpp"

using GameState = oxebots_interfaces::msg::GameState;
using Referee = ssl_league_msgs::msg::Referee;
using BallPosition = oxebots_interfaces::msg::BallPosition;

class RefereeStateNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
    }

    static Referee::SharedPtr makeRefereeMsg(
        uint32_t counter,
        uint8_t command,
        bool blue_positive = false,
        bool yellow_can_place = false,
        bool blue_can_place = false)
    {
        auto msg = std::make_shared<Referee>();
        msg->command_counter = counter;
        msg->command = command;
        msg->stage = 0;
        msg->blue_team_on_positive_half.push_back(blue_positive);
        msg->yellow.goalkeeper = 0;
        msg->yellow.can_place_ball.push_back(yellow_can_place);
        msg->yellow.max_allowed_bots.push_back(3);
        msg->blue.goalkeeper = 0;
        msg->blue.can_place_ball.push_back(blue_can_place);
        msg->blue.max_allowed_bots.push_back(3);
        return msg;
    }

    static BallPosition::SharedPtr makeBallMsg(double x_mm, double y_mm)
    {
        auto msg = std::make_shared<BallPosition>();
        msg->x = x_mm;
        msg->y = y_mm;
        return msg;
    }
};

// ============================================================================
// 1. Transições básicas: HALT -> PREPARE_KICKOFF -> NORMAL_START -> RUN
//    e reset de restart_ctx ao entrar em PHASE_RUN
// ============================================================================

TEST_F(RefereeStateNodeTest, BasicKickoffToRunTransitionAndRestartCtxReset)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("is_yellow_team", false);  // Time azul
    auto node = std::make_shared<RefereeStateNode>(opts);

    // Estado inicial antes de receber GC
    EXPECT_EQ(node->getState().phase, GameState::PHASE_HALT);
    EXPECT_EQ(node->getState().restart_ctx, GameState::RESTART_NONE);
    EXPECT_TRUE(node->getState().stale);
    EXPECT_GT(rclcpp::Time(node->getState().header.stamp).nanoseconds(), 0L);

    // 1) HALT (counter=1)
    node->refereeCallback(makeRefereeMsg(1, Referee::COMMAND_HALT));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_HALT);
    EXPECT_EQ(node->getState().restart_ctx, GameState::RESTART_NONE);
    EXPECT_FALSE(node->getState().stale);

    // 2) PREPARE_KICKOFF_BLUE (counter=2) -> favorecido (somos azul)
    node->refereeCallback(makeRefereeMsg(2, Referee::COMMAND_PREPARE_KICKOFF_BLUE));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_PREPARE_KICKOFF);
    EXPECT_EQ(node->getState().restart_ctx, GameState::RESTART_KICKOFF);
    EXPECT_TRUE(node->getState().we_favored);

    // Posição inicial da bola na origem
    node->ballCallback(makeBallMsg(0.0, 0.0));

    // 3) NORMAL_START (counter=3) -> PHASE_KICKOFF
    node->refereeCallback(makeRefereeMsg(3, Referee::COMMAND_NORMAL_START));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_KICKOFF);
    EXPECT_EQ(node->getState().restart_ctx, GameState::RESTART_KICKOFF);
    EXPECT_TRUE(node->getState().we_favored);

    // Movimento de bola menor que 50 mm mantém PHASE_KICKOFF
    node->ballCallback(makeBallMsg(30.0, 0.0));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_KICKOFF);

    // 4) Movimento de bola >= 50 mm -> transiciona para PHASE_RUN e reseta restart_ctx
    node->ballCallback(makeBallMsg(60.0, 0.0));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_RUN);
    EXPECT_EQ(node->getState().restart_ctx, GameState::RESTART_NONE);
    EXPECT_FALSE(node->getState().we_favored);

    // 5) Se o árbitro enviar STOP e depois NORMAL_START, não deve voltar para KICKOFF
    node->refereeCallback(makeRefereeMsg(4, Referee::COMMAND_STOP));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_STOP);
    EXPECT_EQ(node->getState().restart_ctx, GameState::RESTART_NONE);

    node->refereeCallback(makeRefereeMsg(5, Referee::COMMAND_NORMAL_START));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_RUN);
    EXPECT_EQ(node->getState().restart_ctx, GameState::RESTART_NONE);
}

TEST_F(RefereeStateNodeTest, RestartTimeoutResetsRestartCtx)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("is_yellow_team", false);
    opts.append_parameter_override("kick_timeout_s", 0.01);  // 10 ms para teste rápido
    auto node = std::make_shared<RefereeStateNode>(opts);

    node->refereeCallback(makeRefereeMsg(1, Referee::COMMAND_DIRECT_FREE_BLUE));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_FREE_KICK);
    EXPECT_EQ(node->getState().restart_ctx, GameState::RESTART_FREE_KICK);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // Mesmo command_counter dispara checkRestartTimeout()
    node->refereeCallback(makeRefereeMsg(1, Referee::COMMAND_DIRECT_FREE_BLUE));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_RUN);
    EXPECT_EQ(node->getState().restart_ctx, GameState::RESTART_NONE);
    EXPECT_FALSE(node->getState().we_favored);
}

// ============================================================================
// 2. Deduplicação de command_counter e reinicialização do Game Controller
// ============================================================================

TEST_F(RefereeStateNodeTest, CommandCounterDeduplicationAndGcRestartRecovery)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("is_yellow_team", false);
    opts.append_parameter_override("stale_timeout_ms", 20.0);
    auto node = std::make_shared<RefereeStateNode>(opts);

    // Comando 10: STOP
    node->refereeCallback(makeRefereeMsg(10, Referee::COMMAND_STOP));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_STOP);
    EXPECT_EQ(node->getState().command_counter, 10U);
    EXPECT_FALSE(node->getState().stale);

    // Pacote antigo/fora de ordem (counter=5 < 10) enquanto !stale deve ser ignorado
    node->refereeCallback(makeRefereeMsg(5, Referee::COMMAND_HALT));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_STOP);
    EXPECT_EQ(node->getState().command_counter, 10U);

    // Simula perda de comunicação até ficar stale
    std::this_thread::sleep_for(std::chrono::milliseconds(35));
    node->staleTimerCallback();
    EXPECT_TRUE(node->getState().stale);

    // Game Controller reiniciou com command_counter = 1 (< 10): deve ser aceito!
    node->refereeCallback(makeRefereeMsg(1, Referee::COMMAND_PREPARE_PENALTY_BLUE));
    EXPECT_FALSE(node->getState().stale);
    EXPECT_EQ(node->getState().command_counter, 1U);
    EXPECT_EQ(node->getState().phase, GameState::PHASE_PREPARE_PENALTY);
    EXPECT_EQ(node->getState().restart_ctx, GameState::RESTART_PENALTY);
    EXPECT_TRUE(node->getState().we_favored);
}

// ============================================================================
// 3. Derivação de limites de velocidade e distância mínima por fase
// ============================================================================

TEST_F(RefereeStateNodeTest, DerivedSpeedAndBallDistanceLimitsByPhase)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("is_yellow_team", false);  // Time azul
    auto node = std::make_shared<RefereeStateNode>(opts);

    // HALT: 0.0 m/s, 0.0 mm
    node->refereeCallback(makeRefereeMsg(1, Referee::COMMAND_HALT));
    EXPECT_FLOAT_EQ(node->getState().speed_limit_mps, 0.0f);
    EXPECT_FLOAT_EQ(node->getState().ball_min_distance_mm, 0.0f);

    // STOP: 0.75 m/s, 500.0 mm
    node->refereeCallback(makeRefereeMsg(2, Referee::COMMAND_STOP));
    EXPECT_FLOAT_EQ(node->getState().speed_limit_mps, 0.75f);
    EXPECT_FLOAT_EQ(node->getState().ball_min_distance_mm, 500.0f);

    // TIMEOUT: 0.0 m/s, 0.0 mm
    node->refereeCallback(makeRefereeMsg(3, Referee::COMMAND_TIMEOUT_YELLOW));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_TIMEOUT);
    EXPECT_FLOAT_EQ(node->getState().speed_limit_mps, 0.0f);
    EXPECT_FLOAT_EQ(node->getState().ball_min_distance_mm, 0.0f);

    // PREPARE_KICKOFF: 0.75 m/s, 500.0 mm
    node->refereeCallback(makeRefereeMsg(4, Referee::COMMAND_PREPARE_KICKOFF_YELLOW));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_PREPARE_KICKOFF);
    EXPECT_FALSE(node->getState().we_favored);
    EXPECT_FLOAT_EQ(node->getState().speed_limit_mps, 0.75f);
    EXPECT_FLOAT_EQ(node->getState().ball_min_distance_mm, 500.0f);

    // KICKOFF adversário (NORMAL_START após PREPARE_KICKOFF_YELLOW): 1.5 m/s, 500.0 mm
    node->refereeCallback(makeRefereeMsg(5, Referee::COMMAND_NORMAL_START));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_KICKOFF);
    EXPECT_FALSE(node->getState().we_favored);
    EXPECT_FLOAT_EQ(node->getState().speed_limit_mps, 1.5f);
    EXPECT_FLOAT_EQ(node->getState().ball_min_distance_mm, 500.0f);

    // FREE_KICK nosso (DIRECT_FREE_BLUE): 1.5 m/s, 0.0 mm
    node->refereeCallback(makeRefereeMsg(6, Referee::COMMAND_DIRECT_FREE_BLUE));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_FREE_KICK);
    EXPECT_TRUE(node->getState().we_favored);
    EXPECT_FLOAT_EQ(node->getState().speed_limit_mps, 1.5f);
    EXPECT_FLOAT_EQ(node->getState().ball_min_distance_mm, 0.0f);

    // FREE_KICK adversário (DIRECT_FREE_YELLOW): 1.5 m/s, 500.0 mm
    node->refereeCallback(makeRefereeMsg(7, Referee::COMMAND_DIRECT_FREE_YELLOW));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_FREE_KICK);
    EXPECT_FALSE(node->getState().we_favored);
    EXPECT_FLOAT_EQ(node->getState().speed_limit_mps, 1.5f);
    EXPECT_FLOAT_EQ(node->getState().ball_min_distance_mm, 500.0f);

    // BALL_PLACEMENT nosso com can_place_ball=true: 0.75 m/s, 0.0 mm
    node->refereeCallback(makeRefereeMsg(8, Referee::COMMAND_BALL_PLACEMENT_BLUE, false, false, true));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_BALL_PLACEMENT);
    EXPECT_TRUE(node->getState().we_favored);
    EXPECT_TRUE(node->getState().can_place_ball);
    EXPECT_FLOAT_EQ(node->getState().speed_limit_mps, 0.75f);
    EXPECT_FLOAT_EQ(node->getState().ball_min_distance_mm, 0.0f);

    // BALL_PLACEMENT adversário: 0.75 m/s, 500.0 mm
    node->refereeCallback(makeRefereeMsg(9, Referee::COMMAND_BALL_PLACEMENT_YELLOW, false, true, false));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_BALL_PLACEMENT);
    EXPECT_FALSE(node->getState().we_favored);
    EXPECT_FLOAT_EQ(node->getState().speed_limit_mps, 0.75f);
    EXPECT_FLOAT_EQ(node->getState().ball_min_distance_mm, 500.0f);

    // FORCE_START -> PHASE_RUN: 1.5 m/s, 0.0 mm
    node->refereeCallback(makeRefereeMsg(10, Referee::COMMAND_FORCE_START));
    EXPECT_EQ(node->getState().phase, GameState::PHASE_RUN);
    EXPECT_EQ(node->getState().restart_ctx, GameState::RESTART_FORCE);
    EXPECT_FLOAT_EQ(node->getState().speed_limit_mps, 1.5f);
    EXPECT_FLOAT_EQ(node->getState().ball_min_distance_mm, 0.0f);
}
