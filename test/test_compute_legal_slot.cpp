#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "oxebots_strategy/compute_legal_slot_node.hpp"
#include "oxebots_strategy/defender_nodes.hpp"

namespace
{

class ComputeLegalSlotTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_compute_legal_slot_node");
  }

  void TearDown() override
  {
    node_.reset();
  }

  BT::BehaviorTreeFactory makeFactory()
  {
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<oxebots_strategy::ComputeLegalSlotNode>(
      "ComputeLegalSlot",
      [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<oxebots_strategy::ComputeLegalSlotNode>(name, config, node_);
      });
    return factory;
  }

  BT::Blackboard::Ptr makeStandardBlackboard(bool blue_defends_negative_x = true)
  {
    auto bb = BT::Blackboard::create();
    const double field_length = 4500.0;
    const double field_width = 3000.0;
    const double half_len = field_length / 2.0;

    if (blue_defends_negative_x) {
      bb->set("own_goal_x", -half_len);
      bb->set("opp_goal_x", half_len);
      bb->set("side_sign", 1);
      bb->set("my_goal_x", -half_len);
    } else {
      bb->set("own_goal_x", half_len);
      bb->set("opp_goal_x", -half_len);
      bb->set("side_sign", -1);
      bb->set("my_goal_x", half_len);
    }

    bb->set("field_length", field_length);
    bb->set("field_width", field_width);
    bb->set("penalty_area_depth", 500.0);
    bb->set("penalty_area_width", 1350.0);
    bb->set("center_circle_radius", 500.0);
    bb->set("robot_id", 1u);
    bb->set("is_goalkeeper", false);

    return bb;
  }

  rclcpp::Node::SharedPtr node_;
};

// ============================================================================
// 1. Validações de Entrada e Tratamento de Erros
// ============================================================================

TEST_F(ComputeLegalSlotTest, MissingSlotFails)
{
  auto factory = makeFactory();
  const std::string xml =
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <ComputeLegalSlot slot_x="{slot_x}" slot_y="{slot_y}" slot_w="{slot_w}"/>
         </BehaviorTree>
       </root>)";

  auto bb = makeStandardBlackboard();
  bb->set("robot_x", 0.0);
  bb->set("robot_y", 0.0);
  bb->set("ball_x", 100.0);
  bb->set("ball_y", 0.0);

  auto tree = factory.createTreeFromText(xml, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ComputeLegalSlotTest, UnknownSlotFails)
{
  auto factory = makeFactory();
  const std::string xml =
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <ComputeLegalSlot slot="invalid_slot_name" slot_x="{slot_x}" slot_y="{slot_y}" slot_w="{slot_w}"/>
         </BehaviorTree>
       </root>)";

  auto bb = makeStandardBlackboard();
  bb->set("robot_x", 0.0);
  bb->set("robot_y", 0.0);
  bb->set("ball_x", 100.0);
  bb->set("ball_y", 0.0);

  auto tree = factory.createTreeFromText(xml, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ComputeLegalSlotTest, MissingBallFailsGracefully)
{
  auto factory = makeFactory();
  const std::string xml =
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <ComputeLegalSlot slot="retreat" slot_x="{slot_x}" slot_y="{slot_y}" slot_w="{slot_w}"/>
         </BehaviorTree>
       </root>)";

  auto bb = makeStandardBlackboard();
  bb->set("robot_x", 0.0);
  bb->set("robot_y", 0.0);
  // Não define ball_x nem ball_y nem pose

  auto tree = factory.createTreeFromText(xml, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

// ============================================================================
// 2. Slot 'retreat' (S04 / StopTree / SC-03)
// ============================================================================

TEST_F(ComputeLegalSlotTest, RetreatCalculatesPointAwayFromBall)
{
  auto factory = makeFactory();
  const std::string xml =
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <ComputeLegalSlot slot="retreat" index="0" slot_x="{slot_x}" slot_y="{slot_y}" slot_w="{slot_w}"/>
         </BehaviorTree>
       </root>)";

  auto bb = makeStandardBlackboard();
  // Robô na origem (0, 0), bola a 300 mm em +X
  bb->set("robot_x", 0.0);
  bb->set("robot_y", 0.0);
  bb->set("ball_x", 300.0);
  bb->set("ball_y", 0.0);

  auto tree = factory.createTreeFromText(xml, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  double slot_x = 0.0, slot_y = 0.0, slot_w = 0.0;
  ASSERT_TRUE(bb->get("slot_x", slot_x));
  ASSERT_TRUE(bb->get("slot_y", slot_y));
  ASSERT_TRUE(bb->get("slot_w", slot_w));

  // Distância do slot até a bola deve ser >= 650 mm (500 mm regulamento + 150 mm margem)
  const double dist_to_ball = std::hypot(slot_x - 300.0, slot_y - 0.0);
  EXPECT_GE(dist_to_ball, 649.0);

  // Como o robô estava em (0, 0) e a bola em (300, 0), o recuo deve ser no sentido -X
  EXPECT_LT(slot_x, 0.0);
  EXPECT_NEAR(slot_x, 300.0 - 650.0, 1.0);  // -350 mm
  EXPECT_NEAR(slot_y, 0.0, 1.0);

  // Orientação slot_w deve olhar para a bola (+X, ângulo 0 rad)
  EXPECT_NEAR(slot_w, 0.0, 0.05);
}

TEST_F(ComputeLegalSlotTest, RetreatClampsAwayFromOpponentPenaltyArea)
{
  auto factory = makeFactory();
  const std::string xml =
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <ComputeLegalSlot slot="retreat" index="0" slot_x="{slot_x}" slot_y="{slot_y}" slot_w="{slot_w}"/>
         </BehaviorTree>
       </root>)";

  auto bb = makeStandardBlackboard();
  // Defendemos -2250, gol adversário em +2250
  // Área adversária: X > (2250 - 500) = 1750 mm, Y entre [-675, 675] mm
  // Bola muito perto da área adversária: X = 1700 mm, Y = 0 mm
  // Robô em X = 1800 mm, Y = 0 mm (dentro ou perto da área)
  bb->set("robot_x", 1800.0);
  bb->set("robot_y", 0.0);
  bb->set("ball_x", 1700.0);
  bb->set("ball_y", 0.0);

  auto tree = factory.createTreeFromText(xml, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  double slot_x = 0.0, slot_y = 0.0;
  ASSERT_TRUE(bb->get("slot_x", slot_x));
  ASSERT_TRUE(bb->get("slot_y", slot_y));

  // R-21: Não pode ficar a menos de 200 mm da área adversária
  // Área adversária começa em X = 1750 mm (com margem 220 mm -> X <= 1530 mm ou fora de Y)
  const double opp_area_x_limit = 2250.0 - 500.0 - 200.0; // 1550 mm
  if (std::abs(slot_y) < (1350.0 / 2.0 + 200.0)) {
    EXPECT_LE(slot_x, opp_area_x_limit + 1.0);
  }

  // E a distância para a bola deve ser regulamentar (>= 500 mm)
  const double dist_to_ball = std::hypot(slot_x - 1700.0, slot_y - 0.0);
  EXPECT_GE(dist_to_ball, 499.0);
}

TEST_F(ComputeLegalSlotTest, RetreatRespectsBothSidesOfField)
{
  auto factory = makeFactory();
  const std::string xml =
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <ComputeLegalSlot slot="retreat" index="0" slot_x="{slot_x}" slot_y="{slot_y}" slot_w="{slot_w}"/>
         </BehaviorTree>
       </root>)";

  // Time defendendo o lado positivo (+2250 mm, side_sign = -1)
  auto bb = makeStandardBlackboard(/*blue_defends_negative_x=*/false);
  bb->set("robot_x", 0.0);
  bb->set("robot_y", 0.0);
  bb->set("ball_x", -300.0);
  bb->set("ball_y", 0.0);

  auto tree = factory.createTreeFromText(xml, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  double slot_x = 0.0, slot_y = 0.0;
  ASSERT_TRUE(bb->get("slot_x", slot_x));
  ASSERT_TRUE(bb->get("slot_y", slot_y));

  // Robô recua para longe da bola em direção ao próprio gol (+X)
  EXPECT_GT(slot_x, 0.0);
  EXPECT_NEAR(slot_x, -300.0 + 650.0, 1.0);  // +350 mm
}

// ============================================================================
// 3. Outros Slots Táticos (Cap. 18)
// ============================================================================

TEST_F(ComputeLegalSlotTest, KickoffSlotsOnOwnHalf)
{
  auto factory = makeFactory();
  const std::string xml_kicker =
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="TestKicker">
           <ComputeLegalSlot slot="kickoff_kicker" index="0" slot_x="{slot_x}" slot_y="{slot_y}" slot_w="{slot_w}"/>
         </BehaviorTree>
       </root>)";

  const std::string xml_def =
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="TestDef">
           <ComputeLegalSlot slot="kickoff_def" index="0" slot_x="{slot_x}" slot_y="{slot_y}" slot_w="{slot_w}"/>
         </BehaviorTree>
       </root>)";

  auto bb = makeStandardBlackboard(/*blue_defends_negative_x=*/true);
  bb->set("ball_x", 0.0);
  bb->set("ball_y", 0.0);

  // Kicker no círculo central na própria metade
  auto tree_kicker = factory.createTreeFromText(xml_kicker, bb);
  EXPECT_EQ(tree_kicker.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  double kicker_x = 0.0, kicker_y = 0.0;
  ASSERT_TRUE(bb->get("slot_x", kicker_x));
  ASSERT_TRUE(bb->get("slot_y", kicker_y));
  EXPECT_LT(kicker_x, 0.0);  // Própria metade (-X)
  EXPECT_LT(std::hypot(kicker_x, kicker_y), 500.0);  // Dentro do círculo de raio 500 mm

  // Defensores fora do círculo central na própria metade
  auto tree_def = factory.createTreeFromText(xml_def, bb);
  EXPECT_EQ(tree_def.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  double def_x = 0.0, def_y = 0.0;
  ASSERT_TRUE(bb->get("slot_x", def_x));
  ASSERT_TRUE(bb->get("slot_y", def_y));
  EXPECT_LT(def_x, 0.0);  // Própria metade (-X)
  EXPECT_GE(std::hypot(def_x, def_y), 500.0);  // Fora do círculo central
}

// ============================================================================
// 4. Teste Estrutural da Árvore de STOP (StopSubTree / stop.xml)
// ============================================================================

TEST_F(ComputeLegalSlotTest, StopSubTreeRetreatsWhenBallIsClose)
{
  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<oxebots_strategy::ComputeLegalSlotNode>(
    "ComputeLegalSlot",
    [this](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<oxebots_strategy::ComputeLegalSlotNode>(name, config, node_);
    });

  double recorded_limit = 0.0;
  factory.registerSimpleAction("SetMotionLimit", [&recorded_limit](BT::TreeNode& node) {
    node.getInput("limit_mps", recorded_limit);
    return BT::NodeStatus::SUCCESS;
  }, {BT::InputPort<double>("limit_mps")});

  bool ball_is_close_flag = true;
  factory.registerSimpleCondition("IsBallClose", [&ball_is_close_flag](BT::TreeNode&) {
    return ball_is_close_flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }, {BT::InputPort<unsigned int>("robot_id"), BT::InputPort<double>("distance_threshold")});

  bool goto_called = false;
  factory.registerSimpleAction("GoToPoint", [&goto_called](BT::TreeNode&) {
    goto_called = true;
    return BT::NodeStatus::SUCCESS;
  }, {BT::InputPort<unsigned int>("robot_id"),
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("face_x"),
      BT::InputPort<double>("face_y"),
      BT::InputPort<double>("tolerance")});

  bool hold_called = false;
  factory.registerSimpleAction("HoldPosition", [&hold_called](BT::TreeNode&) {
    hold_called = true;
    return BT::NodeStatus::SUCCESS;
  }, {BT::InputPort<unsigned int>("robot_id")});

  // Estrutura idêntica ao stop.xml
  const std::string stop_xml =
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="StopSubTree">
           <ReactiveSequence name="StopLogic">
             <SetMotionLimit limit_mps="0.75"/>
             <ReactiveFallback name="StopAction">
               <ReactiveSequence name="RetreatIfClose">
                 <IsBallClose robot_id="{robot_id}" distance_threshold="650.0"/>
                 <ComputeLegalSlot slot="retreat" index="0" slot_x="{slot_x}" slot_y="{slot_y}" slot_w="{slot_w}"/>
                 <GoToPoint robot_id="{robot_id}" x="{slot_x}" y="{slot_y}"
                            face_x="{ball_x}" face_y="{ball_y}" tolerance="80.0"/>
               </ReactiveSequence>
               <HoldPosition robot_id="{robot_id}"/>
             </ReactiveFallback>
           </ReactiveSequence>
         </BehaviorTree>
       </root>)";

  auto bb = makeStandardBlackboard();
  bb->set("robot_x", 0.0);
  bb->set("robot_y", 0.0);
  bb->set("ball_x", 200.0);
  bb->set("ball_y", 0.0);

  // Cenário 1: Bola perto (< 650 mm) -> Recua
  ball_is_close_flag = true;
  auto tree_close = factory.createTreeFromText(stop_xml, bb);
  EXPECT_EQ(tree_close.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(recorded_limit, 0.75);
  EXPECT_TRUE(goto_called);
  EXPECT_FALSE(hold_called);

  // Cenário 2: Bola longe (>= 650 mm) -> HoldPosition
  ball_is_close_flag = false;
  goto_called = false;
  hold_called = false;
  auto tree_far = factory.createTreeFromText(stop_xml, bb);
  EXPECT_EQ(tree_far.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(recorded_limit, 0.75);
  EXPECT_FALSE(goto_called);
  EXPECT_TRUE(hold_called);
}

// ============================================================================
// 7. Free Kick Unificado (E07) — Slots freekick_support (SC-07) e wall (SC-08)
// ============================================================================

TEST_F(ComputeLegalSlotTest, FreeKickSupportRespectsOpponentAreaDistance)
{
  auto factory = makeFactory();
  const std::string xml =
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="TestFKSup">
           <ComputeLegalSlot slot="freekick_support" index="0" slot_x="{slot_x}" slot_y="{slot_y}" slot_w="{slot_w}"/>
         </BehaviorTree>
       </root>)";

  auto bb = makeStandardBlackboard(/*blue_defends_negative_x=*/true);
  // Bola perto da área adversária (área adversária começa em x = 2250 - 500 = 1750, largura 1350)
  bb->set("ball_x", 1600.0);
  bb->set("ball_y", 200.0);

  auto tree = factory.createTreeFromText(xml, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  double sx = 0.0, sy = 0.0;
  ASSERT_TRUE(bb->get("slot_x", sx));
  ASSERT_TRUE(bb->get("slot_y", sy));

  // SC-07 / R-21: Deve ficar a >= 200 mm da área adversária e >= 500 mm da bola
  const double opp_area_x_limit = 2250.0 - 500.0; // 1750 mm
  EXPECT_LE(sx, opp_area_x_limit - 200.0 + 1.0);
  const double dist_ball = std::hypot(sx - 1600.0, sy - 200.0);
  EXPECT_GE(dist_ball, 500.0 - 1.0);
}

TEST_F(ComputeLegalSlotTest, FreeKickWallPlacesBetweenBallAndGoalAtLegalDistance)
{
  auto factory = makeFactory();
  const std::string xml =
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="TestWall">
           <ComputeLegalSlot slot="wall" index="0" slot_x="{slot_x}" slot_y="{slot_y}" slot_w="{slot_w}"/>
         </BehaviorTree>
       </root>)";

  auto bb = makeStandardBlackboard(/*blue_defends_negative_x=*/true);
  // Nosso gol em x = -2250, bola em (0, 0)
  bb->set("ball_x", 0.0);
  bb->set("ball_y", 0.0);

  auto tree = factory.createTreeFromText(xml, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  double sx = 0.0, sy = 0.0;
  ASSERT_TRUE(bb->get("slot_x", sx));
  ASSERT_TRUE(bb->get("slot_y", sy));

  // SC-08 / R-13: Barreira deve estar a >= 500 mm da bola em direção ao nosso gol (x negativo)
  EXPECT_LT(sx, 0.0);
  const double dist_ball = std::hypot(sx - 0.0, sy - 0.0);
  EXPECT_GE(dist_ball, 500.0 - 1.0);
}

}  // namespace
