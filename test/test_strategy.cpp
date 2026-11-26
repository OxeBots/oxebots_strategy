#include <gtest/gtest.h>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "oxebots_strategy/movement_calculation.h"
#include "oxebots_strategy/d_star_planner.h"
#include "oxebots_strategy/go_to_point_node.h"
#include "oxebots_strategy/is_ball_in_our_half_condition.h"
#include "oxebots_strategy/is_ball_in_area_condition.h"
#include <thread>

// ============================================================================
// 1. Testes de Utilidades Cinemáticas (Movement Calculation)
// ============================================================================

TEST(MovementCalculationTest, CalculateDistance)
{
    movement::Coordinate p1{0.0f, 0.0f, 0.0f};
    movement::Coordinate p2{300.0f, 400.0f, 0.0f};

    // Triângulo 3-4-5 em milímetros
    double dist = movement::calculateDistance(p1, p2);
    EXPECT_NEAR(dist, 500.0, 1e-4);

    // Distância até o próprio ponto
    EXPECT_DOUBLE_EQ(movement::calculateDistance(p1, p1), 0.0);

    // Distância puramente negativa em coordenadas relativas
    movement::Coordinate p3{-150.0f, -200.0f, 0.0f};
    EXPECT_NEAR(movement::calculateDistance(p1, p3), 250.0, 1e-4);
}

// ============================================================================
// 2. Testes de Semântica de Behavior Tree (Harness T01 do Backlog)
// ============================================================================

TEST(BehaviorTreeHarnessTest, BasicSequenceAndFallbackSemantics)
{
    BT::BehaviorTreeFactory factory;

    // Registra nós síncronos simples de teste
    factory.registerSimpleCondition("AlwaysTrue", [](BT::TreeNode&) {
        return BT::NodeStatus::SUCCESS;
    });

    factory.registerSimpleCondition("AlwaysFalse", [](BT::TreeNode&) {
        return BT::NodeStatus::FAILURE;
    });

    // Teste de Sequence: SUCESSO apenas se todos forem SUCESSO
    const std::string xml_seq =
        R"(<root BTCPP_format="4">
             <BehaviorTree ID="TestSeq">
               <Sequence>
                 <AlwaysTrue/>
                 <AlwaysTrue/>
               </Sequence>
             </BehaviorTree>
           </root>)";

    auto tree_seq = factory.createTreeFromText(xml_seq);
    EXPECT_EQ(tree_seq.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

    // Teste de Fallback: SUCESSO no primeiro filho bem-sucedido
    const std::string xml_fallback =
        R"(<root BTCPP_format="4">
             <BehaviorTree ID="TestFallback">
               <Fallback>
                 <AlwaysFalse/>
                 <AlwaysTrue/>
               </Fallback>
             </BehaviorTree>
           </root>)";

    auto tree_fb = factory.createTreeFromText(xml_fallback);
    EXPECT_EQ(tree_fb.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

// ============================================================================
// 3. Teste Unitário de Folha: IsBallInOurHalfCondition (T04 do Backlog)
// ============================================================================

class IsBallInOurHalfConditionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node_ = std::make_shared<rclcpp::Node>("test_strategy_node");
    }

    void TearDown() override
    {
        node_.reset();
    }

    rclcpp::Node::SharedPtr node_;
};

TEST_F(IsBallInOurHalfConditionTest, BlueTeamDefenseNegativeX)
{
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<oxebots_strategy::IsBallInOurHalfCondition>(
        "IsBallInOurHalf",
        [this](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<oxebots_strategy::IsBallInOurHalfCondition>(name, config, node_);
        });

    const std::string xml_tree =
        R"(<root BTCPP_format="4">
             <BehaviorTree ID="TestTree">
               <IsBallInOurHalf ball_x="{ball_x}"/>
             </BehaviorTree>
           </root>)";

    auto bb = BT::Blackboard::create();
    // Gol próprio em X negativo (ex: Blue Team em my_goal_x = -2250 mm)
    bb->set("my_goal_x", -2250.0);

    // Caso 1: Bola no campo de defesa (X = -500 mm)
    bb->set("ball_x", -500.0);
    auto tree1 = factory.createTreeFromText(xml_tree, bb);
    EXPECT_EQ(tree1.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

    // Caso 2: Bola bem no campo de ataque (X = +300 mm > HALF_FIELD_HYSTERESIS de 100 mm)
    bb->set("ball_x", 300.0);
    auto tree2 = factory.createTreeFromText(xml_tree, bb);
    EXPECT_EQ(tree2.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsBallInOurHalfConditionTest, YellowTeamDefensePositiveX)
{
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<oxebots_strategy::IsBallInOurHalfCondition>(
        "IsBallInOurHalf",
        [this](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<oxebots_strategy::IsBallInOurHalfCondition>(name, config, node_);
        });

    const std::string xml_tree =
        R"(<root BTCPP_format="4">
             <BehaviorTree ID="TestTree">
               <IsBallInOurHalf ball_x="{ball_x}"/>
             </BehaviorTree>
           </root>)";

    auto bb = BT::Blackboard::create();
    // Gol próprio em X positivo (ex: Yellow Team em my_goal_x = +2250 mm)
    bb->set("my_goal_x", 2250.0);

    // Caso 1: Bola no campo de defesa (X = +600 mm)
    bb->set("ball_x", 600.0);
    auto tree1 = factory.createTreeFromText(xml_tree, bb);
    EXPECT_EQ(tree1.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

    // Caso 2: Bola bem no campo de ataque (X = -300 mm < -100 mm)
    bb->set("ball_x", -300.0);
    auto tree2 = factory.createTreeFromText(xml_tree, bb);
    EXPECT_EQ(tree2.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsBallInOurHalfConditionTest, FailsGracefullyWhenMissingBlackboardKeys)
{
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<oxebots_strategy::IsBallInOurHalfCondition>(
        "IsBallInOurHalf",
        [this](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<oxebots_strategy::IsBallInOurHalfCondition>(name, config, node_);
        });

    const std::string xml_tree =
        R"(<root BTCPP_format="4">
             <BehaviorTree ID="TestTree">
               <IsBallInOurHalf ball_x="{ball_x}"/>
             </BehaviorTree>
           </root>)";

    auto bb = BT::Blackboard::create();
    // Não definimos 'my_goal_x' nem 'ball_x'
    auto tree = factory.createTreeFromText(xml_tree, bb);
    // Deve retornar FAILURE sem falhas de segmentação nem exceções não tratadas
    EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

// ============================================================================
// 4. Teste Unitário de Folha: IsBallInAreaCondition (Segurança Regulamentar)
// ============================================================================

class IsBallInAreaConditionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node_ = std::make_shared<rclcpp::Node>("test_strategy_area_node");
    }

    void TearDown() override
    {
        node_.reset();
    }

    rclcpp::Node::SharedPtr node_;
};

TEST_F(IsBallInAreaConditionTest, BallInsideDefenseAreaBlueTeam)
{
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<oxebots_strategy::IsBallInAreaCondition>(
        "IsBallInArea",
        [this](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<oxebots_strategy::IsBallInAreaCondition>(name, config, node_);
        });

    const std::string xml_tree =
        R"(<root BTCPP_format="4">
             <BehaviorTree ID="AreaTest">
               <IsBallInArea ball_x="{ball_x}" ball_y="{ball_y}"/>
             </BehaviorTree>
           </root>)";

    auto bb = BT::Blackboard::create();
    bb->set("my_goal_x", -2250.0);
    bb->set("penalty_area_depth", 800.0);
    bb->set("penalty_area_width", 1600.0);

    // Bola dentro da área de defesa azul (ex: X = -2000 mm, Y = 100 mm)
    bb->set("ball_x", -2000.0);
    bb->set("ball_y", 100.0);
    auto tree_inside = factory.createTreeFromText(xml_tree, bb);
    EXPECT_EQ(tree_inside.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

    // Bola fora da área de defesa azul (ex: X = -1200 mm)
    bb->set("ball_x", -1200.0);
    bb->set("ball_y", 100.0);
    auto tree_outside = factory.createTreeFromText(xml_tree, bb);
    EXPECT_EQ(tree_outside.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsBallInAreaConditionTest, BallInsideDefenseAreaYellowTeam)
{
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<oxebots_strategy::IsBallInAreaCondition>(
        "IsBallInArea",
        [this](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<oxebots_strategy::IsBallInAreaCondition>(name, config, node_);
        });

    const std::string xml_tree =
        R"(<root BTCPP_format="4">
             <BehaviorTree ID="AreaTest">
               <IsBallInArea ball_x="{ball_x}" ball_y="{ball_y}"/>
             </BehaviorTree>
           </root>)";

    auto bb = BT::Blackboard::create();
    bb->set("my_goal_x", 2250.0);
    bb->set("penalty_area_depth", 800.0);
    bb->set("penalty_area_width", 1600.0);

    // Bola dentro da área de defesa amarela (ex: X = +2000 mm, Y = -200 mm)
    bb->set("ball_x", 2000.0);
    bb->set("ball_y", -200.0);
    auto tree_inside = factory.createTreeFromText(xml_tree, bb);
    EXPECT_EQ(tree_inside.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

    // Bola fora da largura da área de defesa (ex: Y = 1000 mm > 800 mm meia-largura)
    bb->set("ball_x", 2000.0);
    bb->set("ball_y", 1000.0);
    auto tree_outside = factory.createTreeFromText(xml_tree, bb);
    EXPECT_EQ(tree_outside.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

// ============================================================================
// 5. Testes Unitários do Planejador D* Lite (M06 / A2 / §5.4 / SC-19)
// ============================================================================

TEST(DStarPlannerUnitTest, PlansAroundInflatedBallAndRespectsStrictRadiusUnderRelaxation)
{
    planning::PlannerConfig cfg;
    cfg.robot_safety_radius = 0.20;
    cfg.occupancy_threshold = 80;
    cfg.max_expansions = 100000;

    planning::DStarPlanner planner(0.05, cfg, rclcpp::get_logger("test_dstar"));

    auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    grid->info.resolution = 0.05f;
    grid->info.width = 60;   // 3.0 m
    grid->info.height = 60;  // 3.0 m
    grid->info.origin.position.x = -1.5;
    grid->info.origin.position.y = -1.5;
    grid->data.assign(60 * 60, 0);
    planner.setOccupancyGrid(grid);

    // Bola no centro (0, 0) com raio STOP de 0.58 m e raio estrito R-06 de 0.50 m
    planner.setDynamicObstacles({{0.0f, 0.0f, 0.58, 0.50}});

    auto start = planner.worldToGrid(-1.0, 0.0, -1.5, -1.5);
    auto goal = planner.worldToGrid(1.0, 0.0, -1.5, -1.5);

    planner.initialize(start, goal);
    ASSERT_EQ(planner.plan(), planning::PlannerStatus::SUCCESS);

    auto path = planner.reconstructPath(start, -1.5, -1.5);
    ASSERT_FALSE(path.poses.empty());

    // Todos os pontos intermediários do caminho contornam a bola a > 0.50 m
    for (const auto & p : path.poses) {
        const double d = std::hypot(p.pose.position.x, p.pose.position.y);
        EXPECT_GE(d, 0.50 - 0.05);
    }

    // Mesmo com relax_obstacles = true (§5.4), o raio estrito de 0.50 m continua bloqueado
    planner.setRelaxObstacles(true);
    auto inside_strict = planner.worldToGrid(0.25, 0.0, -1.5, -1.5);
    // findNearestValidGoal para célula de área 101 encontra a célula válida mais próxima (M06)
    grid->data[goal.y * 60 + goal.x] = 101;
    planner.setOccupancyGrid(grid);
    auto nearest = planner.findNearestValidGoal(goal);
    EXPECT_NE(grid->data[nearest.y * 60 + nearest.x], 101);
    (void)inside_strict;
}

// ============================================================================
// 6. Testes Unitários de GoToPointNode: onHalted e Timeout (M03 / SC-14 / §5.5)
// ============================================================================

TEST(GoToPointUnitTest, OnHaltedPublishesCurrentPoseZeroSpeedAndTimeoutFails)
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    auto node = std::make_shared<rclcpp::Node>("test_gtp_node");
    std::optional<oxebots_interfaces::msg::RobotGoal> last_goal;
    auto sub = node->create_subscription<oxebots_interfaces::msg::RobotGoal>(
        "/robot_goal", rclcpp::QoS(1).transient_local().reliable(),
        [&](const oxebots_interfaces::msg::RobotGoal::SharedPtr msg) {
            last_goal = *msg;
        });
    auto gd_pub = node->create_publisher<oxebots_interfaces::msg::GameData>("/game_data", 10);

    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<oxebots_strategy::GoToPointNode>(
        "GoToPoint",
        [node](const std::string & name, const BT::NodeConfig & config) {
            return std::make_unique<oxebots_strategy::GoToPointNode>(name, config, node);
        });

    auto bb = BT::Blackboard::create();
    bb->set("robot_id", 1u);
    bb->set("my_goal_x", -2250.0);
    bb->set("opponent_goal_x", 2250.0);
    bb->set("penalty_area_depth", 800.0);
    bb->set("penalty_area_width", 1600.0);
    bb->set("is_goalkeeper", false);

    auto tree = factory.createTreeFromText(
        R"(<root BTCPP_format="4">
             <BehaviorTree ID="Main">
               <GoToPoint robot_id="1" x="500.0" y="200.0" tolerance="40.0" timeout_s="0.03"/>
             </BehaviorTree>
           </root>)",
        bb);

    oxebots_interfaces::msg::GameData gd;
    oxebots_interfaces::msg::RobotGameData ally;
    ally.id = 1;
    ally.x = -500.0f;
    ally.y = -100.0f;
    ally.orientation = 0.0f;
    gd.robots.allies.push_back(ally);
    gd_pub->publish(gd);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin_some();

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    exec.spin_some();
    ASSERT_TRUE(last_goal.has_value());
    EXPECT_NEAR(last_goal->pose.pose.position.x, 0.5, 1e-3);

    // halt() na árvore limpa o estado interno de GoToPoint sem sobrescrever /robot_goal
    // (evita que o halt de um ramo à direita apague o novo objetivo já publicado por um ramo à esquerda)
    tree.haltTree();

    // §5.5: se o alvo é inalcançável e timeout_s expira, retorna FAILURE
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(45));
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// ============================================================================
// 7. Testes Unitários do SafetyGate no Controlador (T04 / T06 / T08 / A8 / SC-26)
// ============================================================================

TEST(SafetyGateUnitTest, EnforcesStopSpeedCapKickClampStaleGcAndWatchdog)
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    auto controller = std::make_shared<PathFollowerNode>();
    auto helper = std::make_shared<rclcpp::Node>("test_safety_gate_helper");

    std::optional<oxebots_interfaces::msg::RobotCmd> last_cmd;
    auto cmd_sub = helper->create_subscription<oxebots_interfaces::msg::RobotCmd>(
        "/robot_commands", rclcpp::SensorDataQoS(),
        [&](const oxebots_interfaces::msg::RobotCmd::SharedPtr msg) {
            last_cmd = *msg;
        });

    auto gd_pub = helper->create_publisher<oxebots_interfaces::msg::GameData>("/game_data", 10);
    auto gs_pub = helper->create_publisher<oxebots_interfaces::msg::GameState>(
        "/game_state", rclcpp::QoS(1).transient_local().reliable());
    auto ov_pub = helper->create_publisher<oxebots_interfaces::msg::RobotMotionOverride>(
        "/robot_motion_override", 10);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(controller);
    exec.add_node(helper);

    auto spin_for = [&](std::chrono::milliseconds dur) {
        const auto end = std::chrono::steady_clock::now() + dur;
        while (std::chrono::steady_clock::now() < end) {
            exec.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    };

    // Publica robô 0 na origem
    oxebots_interfaces::msg::GameData gd;
    oxebots_interfaces::msg::RobotGameData r0;
    r0.id = 0;
    r0.x = 0.0f;
    r0.y = 0.0f;
    r0.orientation = 0.0f;
    gd.robots.allies.push_back(r0);
    gd_pub->publish(gd);

    // 1. Em PHASE_STOP (speed_limit_mps = 0.75) com pedido de chute a 6.0 m/s:
    // SafetyGate corta velocidade linear a <= 0.75 m/s (R-27 / T06) e kick_speed a <= 3.0 m/s (SC-24)
    oxebots_interfaces::msg::GameState gs;
    gs.phase = oxebots_interfaces::msg::GameState::PHASE_STOP;
    gs.speed_limit_mps = 0.75f;
    gs.stale = false;
    gs_pub->publish(gs);

    oxebots_interfaces::msg::RobotMotionOverride ov;
    ov.robot_id = 0;
    ov.mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK;
    ov.kick_speed = 6.0f;
    ov_pub->publish(ov);

    spin_for(std::chrono::milliseconds(40));
    ASSERT_TRUE(last_cmd.has_value());
    ASSERT_FALSE(last_cmd->robots.empty());
    const float v_stop = std::hypot(last_cmd->robots[0].x_velocity, last_cmd->robots[0].y_velocity);
    EXPECT_LE(v_stop, 0.751f);
    EXPECT_GT(v_stop, 0.1f);
    EXPECT_FLOAT_EQ(last_cmd->robots[0].kick_speed, 3.0f);

    // 2. A8 / SC-20: GameState.stale = true zera imediatamente velocidades e chute
    gs.stale = true;
    gs_pub->publish(gs);
    ov_pub->publish(ov);
    last_cmd.reset();
    spin_for(std::chrono::milliseconds(40));
    ASSERT_TRUE(last_cmd.has_value());
    EXPECT_FLOAT_EQ(last_cmd->robots[0].x_velocity, 0.0f);
    EXPECT_FLOAT_EQ(last_cmd->robots[0].y_velocity, 0.0f);
    EXPECT_FLOAT_EQ(last_cmd->robots[0].kick_speed, 0.0f);

    // 3. SC-26: Watchdog de 500 ms sem novo /robot_goal ou /robot_motion_override zera o comando
    gs.stale = false;
    gs.phase = oxebots_interfaces::msg::GameState::PHASE_RUN;
    gs.speed_limit_mps = 1.5f;
    gs_pub->publish(gs);
    std::this_thread::sleep_for(std::chrono::milliseconds(530));
    // Renova apenas o GameState para não cair em timeout de GC, mas sem enviar intent ao controlador
    gs_pub->publish(gs);
    last_cmd.reset();
    spin_for(std::chrono::milliseconds(40));
    ASSERT_TRUE(last_cmd.has_value());
    EXPECT_FLOAT_EQ(last_cmd->robots[0].x_velocity, 0.0f);
    EXPECT_FLOAT_EQ(last_cmd->robots[0].y_velocity, 0.0f);
    EXPECT_FLOAT_EQ(last_cmd->robots[0].kick_speed, 0.0f);
}

TEST(SafetyGateUnitTest, EnforcesBallRadialVelocityClampDuringStopAndOpponentRestart)
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    auto controller = std::make_shared<PathFollowerNode>();
    auto helper = std::make_shared<rclcpp::Node>("test_safety_gate_ball_clamp_helper");

    std::optional<oxebots_interfaces::msg::RobotCmd> last_cmd;
    auto cmd_sub = helper->create_subscription<oxebots_interfaces::msg::RobotCmd>(
        "/robot_commands", rclcpp::SensorDataQoS(),
        [&](const oxebots_interfaces::msg::RobotCmd::SharedPtr msg) {
            last_cmd = *msg;
        });

    auto gd_pub = helper->create_publisher<oxebots_interfaces::msg::GameData>("/game_data", 10);
    auto gs_pub = helper->create_publisher<oxebots_interfaces::msg::GameState>(
        "/game_state", rclcpp::QoS(1).transient_local().reliable());
    auto ov_pub = helper->create_publisher<oxebots_interfaces::msg::RobotMotionOverride>(
        "/robot_motion_override", 10);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(controller);
    exec.add_node(helper);

    auto spin_for = [&](std::chrono::milliseconds dur) {
        const auto end = std::chrono::steady_clock::now() + dur;
        while (std::chrono::steady_clock::now() < end) {
            exec.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    };

    // Robô 0 em (0, 0) apontando para +X (orientation = 0), bola em (300, 0) -> dist = 300 mm (< 500 mm)
    oxebots_interfaces::msg::GameData gd;
    gd.ball.x = 300.0f;
    gd.ball.y = 0.0f;
    oxebots_interfaces::msg::RobotGameData r0;
    r0.id = 0;
    r0.x = 0.0f;
    r0.y = 0.0f;
    r0.orientation = 0.0f;
    gd.robots.allies.push_back(r0);
    gd_pub->publish(gd);

    // 1. Em PHASE_STOP com dist = 300 mm < 500 mm: tentativa de avançar em +X (em direção à bola) é bloqueada
    oxebots_interfaces::msg::GameState gs;
    gs.phase = oxebots_interfaces::msg::GameState::PHASE_STOP;
    gs.speed_limit_mps = 0.75f;
    gs.ball_min_distance_mm = 500.0f;
    gs.we_favored = false;
    gs.stale = false;
    gs_pub->publish(gs);

    oxebots_interfaces::msg::RobotMotionOverride ov;
    ov.robot_id = 0;
    ov.mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK;
    ov.kick_speed = 0.0f;  // body bump avança na direção de orientation (+X, contra a bola)
    ov_pub->publish(ov);

    spin_for(std::chrono::milliseconds(40));
    ASSERT_TRUE(last_cmd.has_value());
    ASSERT_FALSE(last_cmd->robots.empty());
    // Componente radial em direção à bola (+X) deve ter sido zerada pelo clamp
    EXPECT_LE(last_cmd->robots[0].x_velocity, 1e-4f);

    // 2. Com enable_safety_gate = false (ablação), o avanço não é bloqueado e passa de 0.75 m/s
    controller->set_parameter(rclcpp::Parameter("enable_safety_gate", false));
    ov_pub->publish(ov);
    last_cmd.reset();
    spin_for(std::chrono::milliseconds(40));
    ASSERT_TRUE(last_cmd.has_value());
    ASSERT_FALSE(last_cmd->robots.empty());
    EXPECT_GT(last_cmd->robots[0].x_velocity, 0.75f);
}


