// E08 — KickOnce: pede o chute, confirma pelo deslocamento da bola e libera o override.
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "oxebots_interfaces/msg/robot_motion_override.hpp"
#include "oxebots_strategy/kick_once_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using Override = oxebots_interfaces::msg::RobotMotionOverride;

// Id sem robô real: se houver uma partida rodando no mesmo domínio ROS, os controladores dela
// ignoram este id e este teste ignora os pedidos dos robôs de verdade.
constexpr unsigned int kTestRobotId = 7;

class KickOnceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_kick_once_node");
    received_.clear();
    sub_ = node_->create_subscription<Override>(
      "/robot_motion_override", rclcpp::QoS(50),
      [this](const Override::SharedPtr msg) {
        if (msg->robot_id == kTestRobotId) {
          received_.push_back(*msg);
        }
      });

    factory_.registerBuilder<oxebots_strategy::KickOnceNode>(
      "KickOnce",
      [this](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<oxebots_strategy::KickOnceNode>(name, config, node_);
      });

    bb_ = BT::Blackboard::create();
    bb_->set("robot_id", kTestRobotId);
    bb_->set("ball_x", 0.0);
    bb_->set("ball_y", 0.0);

    spinFor(150);  // descoberta publicador/assinante
  }

  void TearDown() override
  {
    sub_.reset();
    node_.reset();
  }

  BT::Tree makeTree(const std::string & attrs)
  {
    const std::string xml =
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><KickOnce " + attrs +
      "/></BehaviorTree></root>";
    return factory_.createTreeFromText(xml, bb_);
  }

  void spinFor(int ms)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // Espera até a última mensagem deste robô ter o modo pedido (ou estourar 1 s).
  bool waitForLastMode(uint8_t mode)
  {
    for (int i = 0; i < 200; ++i) {
      rclcpp::spin_some(node_);
      if (!received_.empty() && received_.back().mode == mode) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Override>::SharedPtr sub_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr bb_;
  std::vector<Override> received_;
};

TEST_F(KickOnceTest, PedeOChuteEQuandoABolaSeMoveConfirmaELibera)
{
  auto tree = makeTree("robot_id=\"{robot_id}\" kick_speed=\"2.0\" timeout_s=\"5\"");

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitForLastMode(Override::MODE_KICK));
  EXPECT_DOUBLE_EQ(received_.back().kick_speed, 2.0);

  bb_->set("ball_x", 60.0);  // a bola andou 60 mm
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(waitForLastMode(Override::MODE_NONE)) << "o override deve ser liberado ao confirmar";
}

TEST_F(KickOnceTest, DeslocamentoPequenoNaoConfirma)
{
  auto tree = makeTree("robot_id=\"{robot_id}\" timeout_s=\"5\"");
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  bb_->set("ball_x", 30.0);  // < 50 mm
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  bb_->set("ball_x", 30.0);
  bb_->set("ball_y", 45.0);  // hipotenusa de 54 mm: agora conta
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(KickOnceTest, ReferenciaEABolaNoMomentoDoPedido)
{
  bb_->set("ball_x", 1000.0);  // a bola já estava longe da origem
  auto tree = makeTree("robot_id=\"{robot_id}\" timeout_s=\"5\"");
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  bb_->set("ball_x", 1020.0);  // só 20 mm desde o pedido
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
}

TEST_F(KickOnceTest, LimitaAVelocidadeATresMetrosPorSegundo)
{
  auto tree = makeTree("robot_id=\"{robot_id}\" kick_speed=\"6.5\" timeout_s=\"5\"");
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitForLastMode(Override::MODE_KICK));
  EXPECT_DOUBLE_EQ(received_.back().kick_speed, 3.0);
}

TEST_F(KickOnceTest, SemPortaDeVelocidadeUsaOPadrao)
{
  // Subárvore chamada sem a porta: {kick_speed} não existe na blackboard.
  auto tree = makeTree("robot_id=\"{robot_id}\" kick_speed=\"{kick_speed}\" timeout_s=\"5\"");
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitForLastMode(Override::MODE_KICK));
  EXPECT_DOUBLE_EQ(received_.back().kick_speed, oxebots_strategy::KickOnceNode::kDefaultKickSpeed);
}

TEST_F(KickOnceTest, VelocidadeInvalidaFalha)
{
  auto tree_zero = makeTree("robot_id=\"{robot_id}\" kick_speed=\"0\"");
  EXPECT_EQ(tree_zero.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  auto tree_neg = makeTree("robot_id=\"{robot_id}\" kick_speed=\"-1\"");
  EXPECT_EQ(tree_neg.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(KickOnceTest, EstouraOPrazoSemMoverABolaFalhaELibera)
{
  auto tree = makeTree("robot_id=\"{robot_id}\" timeout_s=\"0.05\"");
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(waitForLastMode(Override::MODE_NONE)) << "o override deve ser liberado no timeout";
}

TEST_F(KickOnceTest, InterromperLiberaOOverride)
{
  auto tree = makeTree("robot_id=\"{robot_id}\" timeout_s=\"5\"");
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitForLastMode(Override::MODE_KICK));

  tree.haltTree();  // outro ramo assumiu (ex.: HALT do árbitro)
  EXPECT_TRUE(waitForLastMode(Override::MODE_NONE)) << "onHalted deve cancelar o chute";
}

TEST_F(KickOnceTest, SemABolaNaBlackboardFalha)
{
  auto bb = BT::Blackboard::create();
  bb->set("robot_id", kTestRobotId);
  const std::string xml =
    "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\">"
    "<KickOnce robot_id=\"{robot_id}\"/></BehaviorTree></root>";
  auto tree = factory_.createTreeFromText(xml, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(KickOnceTest, PrazoOuDeslocamentoInvalidosFalham)
{
  auto tree_a = makeTree("robot_id=\"{robot_id}\" timeout_s=\"0\"");
  EXPECT_EQ(tree_a.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  auto tree_b = makeTree("robot_id=\"{robot_id}\" ball_moved_mm=\"-5\"");
  EXPECT_EQ(tree_b.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

}  // namespace
