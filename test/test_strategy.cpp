#include <gtest/gtest.h>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "oxebots_strategy/movement_calculation.h"
#include "oxebots_strategy/is_ball_in_our_half_condition.h"

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
