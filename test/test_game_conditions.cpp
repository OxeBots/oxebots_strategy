#include <gtest/gtest.h>

#include "behaviortree_cpp/bt_factory.h"
#include "oxebots_strategy/game_conditions.hpp"

namespace
{

BT::BehaviorTreeFactory makeFactory()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<oxebots_strategy::CheckGameStateCondition>("CheckGameState");
  factory.registerNodeType<oxebots_strategy::WeAreFavoredCondition>("WeAreFavored");
  factory.registerNodeType<oxebots_strategy::IsDataFreshCondition>("IsDataFresh");
  factory.registerNodeType<oxebots_strategy::HasRoleCondition>("HasRole");
  return factory;
}

}  // namespace

TEST(GameConditionsTest, CheckGameStateMatchesBlackboardPhase)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("phase", std::string("STOP"));

  const auto tree = factory.createTreeFromText(
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

  const auto tree = factory.createTreeFromText(
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

  const auto different_tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <CheckGameState phase="HALT"/>
         </BehaviorTree>
       </root>)",
    blackboard);
  EXPECT_EQ(different_tree.tickOnce(), BT::NodeStatus::FAILURE);

  auto missing_blackboard = BT::Blackboard::create();
  const auto missing_tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <CheckGameState phase="RUN"/>
         </BehaviorTree>
       </root>)",
    missing_blackboard);
  EXPECT_EQ(missing_tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST(GameConditionsTest, WeAreFavoredReadsBlackboard)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("we_favored", true);

  const auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <WeAreFavored/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  blackboard->set("we_favored", false);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST(GameConditionsTest, IsDataFreshAppliesDocumentedDefaults)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("gc_age_ms", 500.0);
  blackboard->set("pose_age_ms", 150.0);
  blackboard->set("ball_age_ms", 300.0);

  const auto tree = factory.createTreeFromText(
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

TEST(GameConditionsTest, HasRoleMatchesCurrentRole)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set("role", std::string("goalkeeper"));

  const auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <HasRole role="goalkeeper"/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  blackboard->set("role", std::string("defender"));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}
