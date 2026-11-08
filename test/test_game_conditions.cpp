#include <gtest/gtest.h>

#include "behaviortree_cpp/bt_factory.h"
#include "oxebots_strategy/game_conditions.hpp"

namespace
{

BT::BehaviorTreeFactory makeFactory(std::string * last_halt_reason = nullptr)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<oxebots_strategy::CheckGameStateCondition>("CheckGameState");
  factory.registerNodeType<oxebots_strategy::WeAreFavoredCondition>("WeAreFavored");
  factory.registerNodeType<oxebots_strategy::IsDataFreshCondition>("IsDataFresh");
  factory.registerNodeType<oxebots_strategy::HasRoleCondition>("HasRole");
  factory.registerNodeType<oxebots_strategy::IsCornerKickCondition>("IsCornerKick");
  factory.registerNodeType<oxebots_strategy::IsThrowInCondition>("IsThrowIn");

  factory.registerSimpleAction("Halt", [last_halt_reason](BT::TreeNode & node) {
    std::string reason;
    if (node.getInput("reason", reason) && last_halt_reason) {
      *last_halt_reason = reason;
    }
    return BT::NodeStatus::SUCCESS;
  }, {BT::InputPort<std::string>("reason")});

  return factory;
}

}  // namespace

TEST(GameConditionsTest, CheckGameStateMatchesBlackboardPhase)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("phase", std::string("STOP"));

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <CheckGameState phase="STOP"/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST(GameConditionsTest, CheckGameStateAcceptsCurrentNumericPhase)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("phase", 2);

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <CheckGameState phase="STOP"/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST(GameConditionsTest, CheckGameStateFailsForDifferentOrMissingPhase)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("phase", std::string("RUN"));

  auto different_tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <CheckGameState phase="HALT"/>
         </BehaviorTree>
       </root>)",
    blackboard);
  EXPECT_EQ(different_tree.tickOnce(), BT::NodeStatus::FAILURE);

  auto missing_blackboard = BT::Blackboard::create();
  auto missing_tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <CheckGameState phase="RUN"/>
         </BehaviorTree>
       </root>)",
    missing_blackboard);
  EXPECT_EQ(missing_tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Fase numérica fora da faixa [0..9] (ex.: -1 no startup) deve falhar
  auto invalid_id_bb = BT::Blackboard::create();
  invalid_id_bb->set("phase", -1);
  auto invalid_id_tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <CheckGameState phase="HALT"/>
         </BehaviorTree>
       </root>)",
    invalid_id_bb);
  EXPECT_EQ(invalid_id_tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Nome de fase inválido na porta XML deve falhar
  auto invalid_port_tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <CheckGameState phase="FASE_INEXISTENTE"/>
         </BehaviorTree>
       </root>)",
    blackboard);
  EXPECT_EQ(invalid_port_tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST(GameConditionsTest, WeAreFavoredReadsBlackboard)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("we_favored", true);

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <WeAreFavored/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  blackboard->set("we_favored", false);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  auto empty_bb = BT::Blackboard::create();
  auto missing_tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <WeAreFavored/>
         </BehaviorTree>
       </root>)",
    empty_bb);
  EXPECT_EQ(missing_tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST(GameConditionsTest, IsDataFreshAppliesDocumentedDefaults)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("gc_age_ms", 500.0);
  blackboard->set("pose_age_ms", 150.0);
  blackboard->set("ball_age_ms", 300.0);

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <IsDataFresh/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  blackboard->set("ball_age_ms", 300.1);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST(GameConditionsTest, IsDataFreshIgnoresGCAgeWhenRequireGCIsFalseInPort)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  // gc_age_ms estourado
  blackboard->set("gc_age_ms", 99999.0);
  blackboard->set("pose_age_ms", 100.0);
  blackboard->set("ball_age_ms", 200.0);

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <IsDataFresh require_gc="false"/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST(GameConditionsTest, IsDataFreshReadsRequireGCAndLimitsFromBlackboard)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("require_gc", false);
  blackboard->set("max_pose_ms", 250.0);
  blackboard->set("max_ball_ms", 400.0);
  // Sem gc_age_ms na blackboard
  blackboard->set("pose_age_ms", 200.0);
  blackboard->set("ball_age_ms", 350.0);

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <IsDataFresh/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // Se pose_age_ms exceder o limite customizado da Blackboard
  blackboard->set("pose_age_ms", 260.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST(GameConditionsTest, IsDataFreshExplicitPortOverridesBlackboardLimits)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  // Como no strategy_node, a Blackboard tem os limites padrão inicializados
  blackboard->set("max_gc_ms", 500.0);
  blackboard->set("max_pose_ms", 150.0);
  blackboard->set("max_ball_ms", 300.0);
  blackboard->set("gc_age_ms", 100.0);
  blackboard->set("pose_age_ms", 200.0);
  blackboard->set("ball_age_ms", 100.0);

  // Porta explícita max_pose_ms="250.0" no XML deve ter precedência sobre os 150.0 da Blackboard
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <IsDataFresh max_pose_ms="250.0"/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST(GameConditionsTest, HasRoleMatchesCurrentRole)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("role", std::string("goalkeeper"));

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <HasRole role="goalkeeper"/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  blackboard->set("role", std::string("defender"));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Papel ausente na Blackboard ou inválido na porta XML deve falhar
  auto empty_bb = BT::Blackboard::create();
  auto missing_tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <HasRole role="attacker"/>
         </BehaviorTree>
       </root>)",
    empty_bb);
  EXPECT_EQ(missing_tree.tickOnce(), BT::NodeStatus::FAILURE);

  auto invalid_role_tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <HasRole role="papel_invalido"/>
         </BehaviorTree>
       </root>)",
    blackboard);
  EXPECT_EQ(invalid_role_tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST(GameConditionsTest, ConditionsReadRootBlackboardFromInsideSubTreeWithoutAutoremap)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("phase", 4);  // PREPARE_KICKOFF
  blackboard->set("we_favored", true);
  blackboard->set("role", std::string("kicker"));
  blackboard->set("gc_age_ms", 50.0);
  blackboard->set("pose_age_ms", 20.0);
  blackboard->set("ball_age_ms", 30.0);

  const std::string xml_text = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="ChildSubTree">
        <Sequence>
          <IsDataFresh/>
          <CheckGameState phase="PREPARE_KICKOFF"/>
          <WeAreFavored/>
          <HasRole role="kicker"/>
        </Sequence>
      </BehaviorTree>
      <BehaviorTree ID="Main">
        <SubTree ID="ChildSubTree"/>
      </BehaviorTree>
    </root>
  )";

  factory.registerBehaviorTreeFromText(xml_text);
  auto tree = factory.createTree("Main", blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// ============================================================================
// Tarefa E02: HaltTree, TimeoutTree e DegradedModeTree (SC-01, SC-02, SC-13)
// ============================================================================

TEST(SafetyTreesTest, SC01_DegradedModeTreeExecutesOnStaleData)
{
  std::string last_reason;
  auto factory = makeFactory(&last_reason);
  auto blackboard = BT::Blackboard::create();

  // Dados antigos (estourando limite do GC)
  blackboard->set("gc_age_ms", 600.0);
  blackboard->set("pose_age_ms", 50.0);
  blackboard->set("ball_age_ms", 100.0);

  // Ramo SafetyGuard da Master Tree v2 (Capítulo 16 e 17)
  const std::string xml_text = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="DegradedModeTree">
        <Halt reason="stale_data"/>
      </BehaviorTree>
      <BehaviorTree ID="Main">
        <ReactiveSequence name="SafetyGuard">
          <Inverter>
            <IsDataFresh/>
          </Inverter>
          <SubTree ID="DegradedModeTree"/>
        </ReactiveSequence>
      </BehaviorTree>
    </root>
  )";

  factory.registerBehaviorTreeFromText(xml_text);
  auto tree = factory.createTree("Main", blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(last_reason, "stale_data");
}

TEST(SafetyTreesTest, SC02_HaltSubTreeExecutesOnHaltPhase)
{
  std::string last_reason;
  auto factory = makeFactory(&last_reason);
  auto blackboard = BT::Blackboard::create();
  blackboard->set("phase", std::string("HALT"));

  const std::string xml_text = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="HaltSubTree">
        <Halt reason="gc_halt"/>
      </BehaviorTree>
      <BehaviorTree ID="Main">
        <ReactiveSequence name="HaltBranch">
          <CheckGameState phase="HALT"/>
          <SubTree ID="HaltSubTree"/>
        </ReactiveSequence>
      </BehaviorTree>
    </root>
  )";

  factory.registerBehaviorTreeFromText(xml_text);
  auto tree = factory.createTree("Main", blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(last_reason, "gc_halt");
}

TEST(SafetyTreesTest, SC13_TimeoutTreeExecutesOnTimeoutPhase)
{
  std::string last_reason;
  auto factory = makeFactory(&last_reason);
  auto blackboard = BT::Blackboard::create();
  blackboard->set("phase", std::string("TIMEOUT"));

  const std::string xml_text = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TimeoutTree">
        <Halt reason="timeout"/>
      </BehaviorTree>
      <BehaviorTree ID="Main">
        <ReactiveSequence name="TimeoutBranch">
          <CheckGameState phase="TIMEOUT"/>
          <SubTree ID="TimeoutTree"/>
        </ReactiveSequence>
      </BehaviorTree>
    </root>
  )";

  factory.registerBehaviorTreeFromText(xml_text);
  auto tree = factory.createTree("Main", blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(last_reason, "timeout");
}

TEST(GameConditionsTest, IsCornerKickDetectsOffensiveCornersBothSides)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("field_length", 4500.0);
  blackboard->set("field_width", 3000.0);
  blackboard->set("own_goal_x", -2250.0);
  blackboard->set("opp_goal_x", 2250.0);

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <IsCornerKick/>
         </BehaviorTree>
       </root>)",
    blackboard);

  // 1) Escanteio ofensivo superior (+X, +Y) -> SUCCESS
  blackboard->set("ball_x", 2050.0);
  blackboard->set("ball_y", 1350.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // 2) Escanteio ofensivo inferior (+X, -Y) -> SUCCESS
  blackboard->set("ball_x", 2050.0);
  blackboard->set("ball_y", -1350.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // 3) Lateral no meio de campo (x = 200, y = 1400) -> FAILURE
  blackboard->set("ball_x", 200.0);
  blackboard->set("ball_y", 1400.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // 4) Falta frontal perto da área (x = 1400, y = 100) -> FAILURE
  blackboard->set("ball_x", 1400.0);
  blackboard->set("ball_y", 100.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // 5) Quina defensiva (-X, +Y) quando atacamos +X -> FAILURE
  blackboard->set("ball_x", -2050.0);
  blackboard->set("ball_y", 1350.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // 6) Troca de lados: atacando -X (opp_goal_x = -2250), quina (-2050, 1350) passa a ser escanteio ofensivo -> SUCCESS
  blackboard->set("own_goal_x", 2250.0);
  blackboard->set("opp_goal_x", -2250.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST(GameConditionsTest, IsThrowInDetectsSidelineOutsideCornersBothSides)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("field_length", 4500.0);
  blackboard->set("field_width", 3000.0);

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <IsThrowIn/>
         </BehaviorTree>
       </root>)",
    blackboard);

  // 1) Lateral superior no meio de campo (x = 200, y = 1350) -> SUCCESS
  blackboard->set("ball_x", 200.0);
  blackboard->set("ball_y", 1350.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // 2) Lateral inferior no campo de defesa (x = -800, y = -1350) -> SUCCESS
  blackboard->set("ball_x", -800.0);
  blackboard->set("ball_y", -1350.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // 3) Quina de escanteio (+2050, +1350) -> FAILURE (é escanteio, não lateral)
  blackboard->set("ball_x", 2050.0);
  blackboard->set("ball_y", 1350.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // 4) Centro do campo (x = 0, y = 200) -> FAILURE
  blackboard->set("ball_x", 0.0);
  blackboard->set("ball_y", 200.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}
