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

// ============================================================================
// 5. Observabilidade de Transição de Ramo (/strategy/active_branch) — A04
// ============================================================================

TEST_F(StrategyNodeBlackboardTest, BranchTransitionPublishesAndUpdatesActiveBranch)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("is_yellow_team", false);
    opts.append_parameter_override("robot_id", 1);
    opts.append_parameter_override("bt_xml_path", "master_strategy_el.xml");

    auto node = std::make_shared<StrategyNode>(opts);
    ASSERT_TRUE(node->init());

    std::vector<std::string> published_transitions;
    auto sub_node = std::make_shared<rclcpp::Node>("test_active_branch_listener");
    auto sub = sub_node->create_subscription<std_msgs::msg::String>(
        "/strategy/active_branch",
        rclcpp::QoS(10).transient_local().reliable(),
        [&published_transitions](const std_msgs::msg::String::SharedPtr msg) {
            published_transitions.push_back(msg->data);
        });
    auto gd_pub = sub_node->create_publisher<GameData>("/game_data", 10);

    auto spin_both = [&]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node);
        exec.add_node(sub_node);
        for (int i = 0; i < 5; ++i) {
            exec.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    };

    for (int i = 0; i < 50 && (gd_pub->get_subscription_count() < 3 || sub->get_publisher_count() < 1); ++i) {
        spin_both();
    }

    node->setLastGeometry(makeGeometry(4500.0, 3000.0));

    auto roles = std::make_shared<RoleAssignment>();
    roles->attacker_id = 2;
    roles->defender_id = 1;
    node->setLastRoleAssignment(roles);

    auto gd = std::make_shared<GameData>();
    gd->ball.x = 0.0f;
    gd->ball.y = 0.0f;
    oxebots_interfaces::msg::RobotGameData ally;
    ally.id = 1;
    ally.x = -1200.0f;
    ally.y = 0.0f;
    ally.orientation = 0.0f;
    gd->robots.allies.push_back(ally);
    node->setLastGameData(gd);
    gd_pub->publish(*gd);
    spin_both();

    // Tick 1: sem GameState (gc_age_ms = 1e9) -> SafetyGuard (stale_gc)
    node->refresh();
    node->tickOnce();
    spin_both();

    EXPECT_EQ(node->getActiveBranch(), "SafetyGuard");
    ASSERT_EQ(published_transitions.size(), 1u);
    EXPECT_EQ(
        published_transitions.back(),
        "robot_id=1 branch=SafetyGuard prev_branch=NONE phase=HALT role=defender reason=stale_gc");

    // Tick 2: permanece em SafetyGuard -> nenhuma nova mensagem publicada
    node->setLastGameData(gd);
    node->refresh();
    node->tickOnce();
    spin_both();

    EXPECT_EQ(node->getActiveBranch(), "SafetyGuard");
    EXPECT_EQ(published_transitions.size(), 1u);

    // Tick 3: chega GameState fresco com PHASE_STOP -> transição SafetyGuard -> StopBranch
    auto gs = makeGameState(false);
    gs->header.stamp = node->now();
    gs->phase = GameState::PHASE_STOP;
    node->setLastGameState(gs);
    node->setLastGameData(gd);
    node->refresh();
    node->tickOnce();
    spin_both();

    EXPECT_EQ(node->getActiveBranch(), "StopBranch");
    ASSERT_EQ(published_transitions.size(), 2u);
    EXPECT_EQ(
        published_transitions.back(),
        "robot_id=1 branch=StopBranch prev_branch=SafetyGuard phase=STOP role=defender reason=data_fresh,phase=STOP");

    // Tick 4: continua em PHASE_STOP (ramo assíncrono em RUNNING) -> não republica
    gs->header.stamp = node->now();
    node->setLastGameState(gs);
    node->setLastGameData(gd);
    node->refresh();
    node->tickOnce();
    spin_both();

    EXPECT_EQ(node->getActiveBranch(), "StopBranch");
    EXPECT_EQ(published_transitions.size(), 2u);

    // Tick 5: muda para PHASE_HALT -> transição StopBranch -> HaltBranch
    gs->header.stamp = node->now();
    gs->phase = GameState::PHASE_HALT;
    node->setLastGameState(gs);
    node->setLastGameData(gd);
    node->refresh();
    node->tickOnce();
    spin_both();

    EXPECT_EQ(node->getActiveBranch(), "HaltBranch");
    ASSERT_EQ(published_transitions.size(), 3u);
    EXPECT_EQ(
        published_transitions.back(),
        "robot_id=1 branch=HaltBranch prev_branch=StopBranch phase=HALT role=defender reason=phase=HALT");

    // Tick 6: muda para PHASE_RUN -> transição HaltBranch -> RunBranch
    gs->header.stamp = node->now();
    gs->phase = GameState::PHASE_RUN;
    node->setLastGameState(gs);
    node->setLastGameData(gd);
    node->refresh();
    node->tickOnce();
    spin_both();

    EXPECT_EQ(node->getActiveBranch(), "RunBranch");
    ASSERT_EQ(published_transitions.size(), 4u);
    EXPECT_EQ(
        published_transitions.back(),
        "robot_id=1 branch=RunBranch prev_branch=HaltBranch phase=RUN role=defender reason=phase=RUN");
}

// ============================================================================
// 6. Registro Único de Nós e TreeNodesModel para Groot2 — A05
// ============================================================================

TEST_F(StrategyNodeBlackboardTest, NodeRegistryAndTreeNodesModelForGroot2)
{
    auto ros_node = std::make_shared<rclcpp::Node>("test_node_registry");
    BT::BehaviorTreeFactory factory;
    oxebots_strategy::registerStrategyNodes(factory, ros_node);

    const std::string generated_xml = oxebots_strategy::generateTreeNodesModelXML(factory, false);
    EXPECT_NE(generated_xml.find("<TreeNodesModel>"), std::string::npos);

    const auto & builtin = factory.builtinNodes();
    const auto & manifests = factory.manifests();

    const std::string share_dir = ament_index_cpp::get_package_share_directory("oxebots_strategy");
    const fs::path model_path = fs::path(share_dir) / "behavior_trees" / "tree_nodes_model.xml";
    ASSERT_TRUE(fs::exists(model_path)) << "Arquivo ausente: " << model_path.string();

    std::ifstream file_in(model_path);
    const std::string file_xml(
        (std::istreambuf_iterator<char>(file_in)),
        std::istreambuf_iterator<char>());
    EXPECT_NE(file_xml.find("<TreeNodesModel>"), std::string::npos);

    size_t custom_count = 0;
    for (const auto & [id, manifest] : manifests) {
        (void)manifest;
        if (builtin.count(id) > 0) {
            continue;
        }
        ++custom_count;
        const std::string id_attr = "ID=\"" + id + "\"";
        EXPECT_NE(generated_xml.find(id_attr), std::string::npos)
            << "Nó ausente no XML gerado: " << id;
        EXPECT_NE(file_xml.find(id_attr), std::string::npos)
            << "Nó ausente em tree_nodes_model.xml: " << id;
    }
    EXPECT_GE(custom_count, 32u);
}

// ============================================================================
// 7. Free Kick Unificado (E07) — Transição e Roteamento de FreeKickBranch
// ============================================================================

TEST_F(StrategyNodeBlackboardTest, FreeKickBranchTransitionsOurAndTheir)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("is_yellow_team", false);
    opts.append_parameter_override("robot_id", 1);
    opts.append_parameter_override("bt_xml_path", "master_strategy_el.xml");

    auto node = std::make_shared<StrategyNode>(opts);
    ASSERT_TRUE(node->init());

    auto pub_node = std::make_shared<rclcpp::Node>("test_fk_pub");
    auto gd_pub = pub_node->create_publisher<GameData>("/game_data", 10);
    auto spin_both = [&]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node);
        exec.add_node(pub_node);
        for (int i = 0; i < 5; ++i) {
            exec.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    };
    for (int i = 0; i < 30 && gd_pub->get_subscription_count() < 1; ++i) {
        spin_both();
    }

    node->setLastGeometry(makeGeometry(4500.0, 3000.0));

    auto roles = std::make_shared<RoleAssignment>();
    roles->goalkeeper_id = 0;
    roles->attacker_id = 1;
    roles->defender_id = 2;
    roles->kicker_id = 1;
    node->setLastRoleAssignment(roles);

    auto gd = std::make_shared<GameData>();
    gd->ball.x = 200.0f;
    gd->ball.y = -300.0f;
    oxebots_interfaces::msg::RobotGameData ally;
    ally.id = 1;
    ally.x = 0.0f;
    ally.y = 0.0f;
    ally.orientation = 0.0f;
    gd->robots.allies.push_back(ally);
    node->setLastGameData(gd);
    gd_pub->publish(*gd);
    spin_both();

    // 1) Free Kick Nosso (we_favored = true) -> Kicker executa RestartKickerTree (sem criar barreira)
    node->getBlackboard()->unset("slot_x");
    node->getBlackboard()->unset("slot_y");
    auto gs_our = makeGameState(false, GameState::PHASE_FREE_KICK, /*we_favored=*/true);
    gs_our->header.stamp = node->now();
    node->setLastGameState(gs_our);
    node->refresh();
    node->tickOnce();
    spin_both();

    EXPECT_EQ(node->getActiveBranch(), "FreeKickBranch");
    double dummy_x = 0.0;
    // Kicker executa RestartKickerTree, não o slot de barreira (wall)
    EXPECT_FALSE(node->getBlackboard()->get("slot_x", dummy_x));

    // 2) Free Kick Deles (we_favored = false) -> Linha forma WallRole a >= 500 mm da bola
    auto gs_their = makeGameState(false, GameState::PHASE_FREE_KICK, /*we_favored=*/false);
    gs_their->header.stamp = node->now();
    node->setLastGameState(gs_their);
    node->refresh();
    node->tickOnce();
    spin_both();

    EXPECT_EQ(node->getActiveBranch(), "FreeKickBranch");
    double wall_x = 0.0, wall_y = 0.0;
    ASSERT_TRUE(node->getBlackboard()->get("slot_x", wall_x));
    ASSERT_TRUE(node->getBlackboard()->get("slot_y", wall_y));
    const double dist_wall_ball = std::hypot(wall_x - 200.0, wall_y - (-300.0));
    EXPECT_GE(dist_wall_ball, 500.0 - 1.0);
}

// ============================================================================
// 8. Kick-off Unificado (E05) — Transição de PrepareKickoffBranch e KickoffBranch
// ============================================================================

TEST_F(StrategyNodeBlackboardTest, PrepareKickoffAndKickoffBranchesTransitionOurAndTheir)
{
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("is_yellow_team", false);
    opts.append_parameter_override("robot_id", 1);
    opts.append_parameter_override("bt_xml_path", "master_strategy_el.xml");

    auto node = std::make_shared<StrategyNode>(opts);
    ASSERT_TRUE(node->init());

    auto pub_node = std::make_shared<rclcpp::Node>("test_ko_pub");
    auto gd_pub = pub_node->create_publisher<GameData>("/game_data", 10);
    auto spin_both = [&]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node);
        exec.add_node(pub_node);
        for (int i = 0; i < 5; ++i) {
            exec.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    };
    for (int i = 0; i < 30 && gd_pub->get_subscription_count() < 1; ++i) {
        spin_both();
    }

    node->setLastGeometry(makeGeometry(4500.0, 3000.0));

    auto roles = std::make_shared<RoleAssignment>();
    roles->goalkeeper_id = 0;
    roles->attacker_id = 1;
    roles->defender_id = 2;
    roles->kicker_id = 1;
    node->setLastRoleAssignment(roles);

    auto gd = std::make_shared<GameData>();
    gd->ball.x = 0.0f;
    gd->ball.y = 0.0f;
    oxebots_interfaces::msg::RobotGameData ally;
    ally.id = 1;
    ally.x = -280.0f;
    ally.y = 0.0f;
    ally.orientation = 0.0f;
    gd->robots.allies.push_back(ally);
    node->setLastGameData(gd);
    gd_pub->publish(*gd);
    spin_both();

    // 1) PREPARE_KICKOFF Nosso (we_favored = true) -> Cobrador vai para o círculo central
    auto gs_prep_our = makeGameState(false, GameState::PHASE_PREPARE_KICKOFF, /*we_favored=*/true);
    gs_prep_our->header.stamp = node->now();
    node->setLastGameState(gs_prep_our);
    node->refresh();
    node->tickOnce();
    spin_both();
    EXPECT_EQ(node->getActiveBranch(), "PrepareKickoffBranch");
    double prep_our_x = 0.0;
    ASSERT_TRUE(node->getBlackboard()->get("slot_x", prep_our_x));
    EXPECT_LT(prep_our_x, 0.0);
    EXPECT_LE(std::abs(prep_our_x), 500.0); // No círculo central (SC-05)

    // 2) PREPARE_KICKOFF Deles (we_favored = false) -> Defensor fica fora do círculo central
    auto gs_prep_their = makeGameState(false, GameState::PHASE_PREPARE_KICKOFF, /*we_favored=*/false);
    gs_prep_their->header.stamp = node->now();
    node->setLastGameState(gs_prep_their);
    node->refresh();
    node->tickOnce();
    spin_both();
    EXPECT_EQ(node->getActiveBranch(), "PrepareKickoffBranch");
    double prep_their_x = 0.0;
    ASSERT_TRUE(node->getBlackboard()->get("slot_x", prep_their_x));
    EXPECT_LT(prep_their_x, 0.0);
    EXPECT_GE(std::abs(prep_their_x), 500.0); // Fora do círculo central (SC-06)

    // 3) KICKOFF Nosso (we_favored = true) -> Cobrador executa RestartKickerTree (sem slot de espera)
    node->getBlackboard()->unset("slot_x");
    auto gs_kick_our = makeGameState(false, GameState::PHASE_KICKOFF, /*we_favored=*/true);
    gs_kick_our->header.stamp = node->now();
    node->setLastGameState(gs_kick_our);
    node->refresh();
    node->tickOnce();
    spin_both();
    EXPECT_EQ(node->getActiveBranch(), "KickoffBranch");
    double kick_slot_x = 0.0;
    // Cobrador executa RestartKickerTree, não o slot de espera defensiva
    EXPECT_FALSE(node->getBlackboard()->get("slot_x", kick_slot_x));

    // 4) KICKOFF Deles (we_favored = false) -> Permanece em espera legal fora do círculo (slot_x <= -500)
    auto gs_kick_their = makeGameState(false, GameState::PHASE_KICKOFF, /*we_favored=*/false);
    gs_kick_their->header.stamp = node->now();
    node->setLastGameState(gs_kick_their);
    node->refresh();
    node->tickOnce();
    spin_both();
    EXPECT_EQ(node->getActiveBranch(), "KickoffBranch");
    double kick_their_x = 0.0;
    ASSERT_TRUE(node->getBlackboard()->get("slot_x", kick_their_x));
    EXPECT_LT(kick_their_x, 0.0);
    EXPECT_GE(std::abs(kick_their_x), 500.0);
}

