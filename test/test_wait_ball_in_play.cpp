// E08 — WaitBallInPlay: regra de "bola em jogo" (R-16): bola moveu >= moved_mm OU passou timeout_s.
#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "oxebots_strategy/wait_ball_in_play_condition.hpp"

namespace
{

using oxebots_strategy::WaitBallInPlayCondition;

class WaitBallInPlayTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    factory_.registerNodeType<WaitBallInPlayCondition>("WaitBallInPlay");
    bb_ = BT::Blackboard::create();
    bb_->set("ball_x", 0.0);
    bb_->set("ball_y", 0.0);
    bb_->set("phase", 5);  // KICKOFF
  }

  BT::Tree makeTree(const std::string & attrs = "")
  {
    const std::string xml =
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><WaitBallInPlay " + attrs +
      "/></BehaviorTree></root>";
    return factory_.createTreeFromText(xml, bb_);
  }

  void moveBallTo(double x, double y = 0.0)
  {
    bb_->set("ball_x", x);
    bb_->set("ball_y", y);
  }

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr bb_;
};

TEST_F(WaitBallInPlayTest, BolaParadaNaoEstaEmJogo)
{
  auto tree = makeTree();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(WaitBallInPlayTest, MoverMenosQueOLimiteNaoBasta_MoverOLimiteColoca)
{
  auto tree = makeTree();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);  // registra a referência (0, 0)

  moveBallTo(49.0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  moveBallTo(50.0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(WaitBallInPlayTest, DistanciaEEuclidiana)
{
  auto tree = makeTree();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  // 36 mm em x e 36 mm em y: 50,9 mm de deslocamento, nenhum eixo sozinho chega a 50.
  moveBallTo(36.0, 36.0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(WaitBallInPlayTest, DepoisDeEmJogoNaoVoltaAtras)
{
  auto tree = makeTree();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  moveBallTo(80.0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  // A bola volta para perto da referência: continua em jogo (a regra mede o início, não a posição).
  moveBallTo(1.0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(WaitBallInPlayTest, TimeoutColocaEmJogoMesmoComABolaParada)
{
  auto tree = makeTree("timeout_s=\"0.05\"");
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(WaitBallInPlayTest, MudancaDeFaseIniciaEpisodioNovo)
{
  auto tree = makeTree();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  moveBallTo(80.0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  // Nova fase (pênalti): a bola está em (80, 0) e a referência passa a ser essa posição.
  bb_->set("phase", 7);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  moveBallTo(120.0);  // 40 mm da nova referência
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  moveBallTo(135.0);  // 55 mm da nova referência
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(WaitBallInPlayTest, RamoInterrompidoPorMuitoTempoIniciaEpisodioNovo)
{
  auto tree = makeTree();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  moveBallTo(80.0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  // A condição fica sem ser avaliada por mais que a lacuna de episódio: um reinício novo.
  std::this_thread::sleep_for(std::chrono::milliseconds(
    static_cast<int>(WaitBallInPlayCondition::kEpisodeGapS * 1000.0) + 100));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(WaitBallInPlayTest, SemABolaNaBlackboardFalha)
{
  auto bb = BT::Blackboard::create();
  bb->set("phase", 5);
  const std::string xml =
    "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><WaitBallInPlay/></BehaviorTree></root>";
  auto tree = factory_.createTreeFromText(xml, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(WaitBallInPlayTest, ParametrosInvalidosFalham)
{
  auto tree_a = makeTree("moved_mm=\"0\"");
  EXPECT_EQ(tree_a.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  auto tree_b = makeTree("timeout_s=\"-1\"");
  EXPECT_EQ(tree_b.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(WaitBallInPlayTest, SemFaseSoReiniciaPelaLacuna)
{
  auto bb = BT::Blackboard::create();
  bb->set("ball_x", 0.0);
  bb->set("ball_y", 0.0);
  const std::string xml =
    "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><WaitBallInPlay/></BehaviorTree></root>";
  auto tree = factory_.createTreeFromText(xml, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  bb->set("ball_x", 60.0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

}  // namespace
