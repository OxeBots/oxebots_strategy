// E10 — defensor agnóstico de lado: IsBallInOpponentField por my_goal_x, alvos fora da pequena área
// (SC-17) e troca de papel no mesmo tick (SC-15).
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_strategy/defender_nodes.hpp"
#include "oxebots_strategy/game_conditions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using RobotGoal = oxebots_interfaces::msg::RobotGoal;

// Id sem robô real (ver test_kick_once.cpp).
constexpr uint32_t kTestRobotId = 7;
constexpr double kFieldHalfLength = 2250.0;
constexpr double kPenaltyDepth = 500.0;
constexpr double kPenaltyWidth = 1000.0;
constexpr double kFieldWidth = 3000.0;
constexpr double kAreaMargin = 200.0;  // margem do clamp do defensor (defender_nodes.cpp)

// Probe do ramo do atacante: fica em RUNNING e conta quantas vezes foi interrompido.
class FakeAttack : public BT::StatefulActionNode
{
public:
  FakeAttack(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override {return BT::NodeStatus::RUNNING;}
  BT::NodeStatus onRunning() override {return BT::NodeStatus::RUNNING;}
  void onHalted() override {++halts;}
  static inline int halts = 0;
};

// Probe do ramo do defensor: sincrono, conta quantas vezes foi executado.
class FakeDefend : public BT::SyncActionNode
{
public:
  FakeDefend(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override {++ticks; return BT::NodeStatus::SUCCESS;}
  static inline int ticks = 0;
};

class DefenderNodesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_defender_nodes_node");
    sub_ = node_->create_subscription<RobotGoal>(
      "/robot_goal", rclcpp::QoS(rclcpp::KeepLast(50)).transient_local().reliable(),
      [this](const RobotGoal::SharedPtr msg) {
        if (msg->robot_id == kTestRobotId) {
          last_goal_ = *msg;
        }
      });
    configureDefenderController(node_, kTestRobotId);

    factory_.registerNodeType<IsBallInOpponentField>("IsBallInOpponentField");
    factory_.registerNodeType<ShadowBall>("ShadowBall");
    factory_.registerNodeType<MarkOpponent>("MarkOpponent");
    factory_.registerNodeType<oxebots_strategy::HasRoleCondition>("HasRole");
    factory_.registerNodeType<FakeAttack>("FakeAttack");
    factory_.registerNodeType<FakeDefend>("FakeDefend");
    FakeAttack::halts = 0;
    FakeDefend::ticks = 0;

    bb_ = BT::Blackboard::create();
    spinFor(150);
    // O /robot_goal é transient_local: o objetivo de um teste anterior pode chegar na descoberta.
    // Descarta-o aqui para que cada teste só veja o que ele mesmo publicou.
    last_goal_.reset();
  }

  void TearDown() override
  {
    sub_.reset();
    node_.reset();
  }

  void spinFor(int ms)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  BT::Tree makeTree(const std::string & body)
  {
    const std::string xml =
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\">" + body + "</BehaviorTree></root>";
    return factory_.createTreeFromText(xml, bb_);
  }

  void setDefenseBlackboard(double my_goal_x, bool with_depth = true)
  {
    bb_->set("my_goal_x", my_goal_x);
    if (with_depth) {
      bb_->set("penalty_area_depth", kPenaltyDepth);
    }
    bb_->set("penalty_area_width", kPenaltyWidth);
    bb_->set("field_width", kFieldWidth);
  }

  // Posição X (mm) do último objetivo publicado, depois de dar tempo à entrega.
  std::optional<double> publishedGoalX()
  {
    for (int i = 0; i < 100 && !last_goal_.has_value(); ++i) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!last_goal_.has_value()) {
      return std::nullopt;
    }
    return last_goal_->pose.pose.position.x * 1000.0;
  }

  // Última posição (x, y em mm) publicada; descarta-a antes, para o teste só ver o que ele publicou.
  std::optional<std::pair<double, double>> tickAndGetGoal(BT::Tree & tree)
  {
    last_goal_.reset();
    if (tree.tickExactlyOnce() != BT::NodeStatus::SUCCESS) {
      return std::nullopt;
    }
    for (int i = 0; i < 100 && !last_goal_.has_value(); ++i) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!last_goal_.has_value()) {
      return std::nullopt;
    }
    return std::make_pair(last_goal_->pose.pose.position.x * 1000.0,
                          last_goal_->pose.pose.position.y * 1000.0);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<RobotGoal>::SharedPtr sub_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr bb_;
  std::optional<RobotGoal> last_goal_;
};

// ----------------------------------------------------------------------------------------------
// IsBallInOpponentField: o lado vem do gol próprio, não da cor
// ----------------------------------------------------------------------------------------------

TEST_F(DefenderNodesTest, CampoAdversarioComGolProprioEmXNegativo)
{
  bb_->set("my_goal_x", -kFieldHalfLength);
  auto tree = makeTree("<IsBallInOpponentField ball_x=\"{ball_x}\"/>");

  bb_->set("ball_x", 500.0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  bb_->set("ball_x", -500.0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(DefenderNodesTest, CampoAdversarioComGolProprioEmXPositivo)
{
  bb_->set("my_goal_x", kFieldHalfLength);
  auto tree = makeTree("<IsBallInOpponentField ball_x=\"{ball_x}\"/>");

  bb_->set("ball_x", -500.0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  bb_->set("ball_x", 500.0);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(DefenderNodesTest, TrocaDeLadosInverteOResultadoSemMudarAArvore)
{
  auto tree = makeTree("<IsBallInOpponentField ball_x=\"{ball_x}\"/>");
  bb_->set("ball_x", 800.0);

  bb_->set("my_goal_x", -kFieldHalfLength);
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);

  bb_->set("my_goal_x", kFieldHalfLength);  // segundo tempo: lados trocados
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(DefenderNodesTest, CorLegadaIgnoradaQuandoHaMyGoalX)
{
  bb_->set("my_goal_x", -kFieldHalfLength);
  bb_->set("ball_x", 500.0);
  // is_yellow="true" diria "campo adversário é x < 0" (regra antiga); my_goal_x manda.
  auto tree = makeTree("<IsBallInOpponentField ball_x=\"{ball_x}\" is_yellow=\"true\"/>");
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(DefenderNodesTest, SemMyGoalXUsaACorLegadaESemNenhumFalha)
{
  bb_->set("ball_x", 500.0);
  auto legacy = makeTree("<IsBallInOpponentField ball_x=\"{ball_x}\" is_yellow=\"false\"/>");
  EXPECT_EQ(legacy.tickExactlyOnce(), BT::NodeStatus::SUCCESS);  // azul antigo: x > 0 é adversário

  auto none = makeTree("<IsBallInOpponentField ball_x=\"{ball_x}\"/>");
  EXPECT_EQ(none.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

// ----------------------------------------------------------------------------------------------
// SC-17: o defensor nunca mira dentro da pequena área
// ----------------------------------------------------------------------------------------------

TEST_F(DefenderNodesTest, ShadowBallFicaForaDaAreaComGolEmXNegativo)
{
  setDefenseBlackboard(-kFieldHalfLength);
  bb_->set("ball", Pose2D{-2000.0, 0.0, 0.0});  // bola colada no nosso gol
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  auto tree = makeTree("<ShadowBall ball=\"{ball}\" ball_vel=\"{ball_vel}\" my_goal_x=\"{my_goal_x}\"/>");

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  const auto x = publishedGoalX();
  ASSERT_TRUE(x.has_value());
  EXPECT_GE(*x, -kFieldHalfLength + kPenaltyDepth + kAreaMargin - 1.0);
}

TEST_F(DefenderNodesTest, ShadowBallFicaForaDaAreaComGolEmXPositivo)
{
  setDefenseBlackboard(kFieldHalfLength);
  bb_->set("ball", Pose2D{2000.0, 0.0, 0.0});
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  auto tree = makeTree("<ShadowBall ball=\"{ball}\" ball_vel=\"{ball_vel}\" my_goal_x=\"{my_goal_x}\"/>");

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  const auto x = publishedGoalX();
  ASSERT_TRUE(x.has_value());
  EXPECT_LE(*x, kFieldHalfLength - kPenaltyDepth - kAreaMargin + 1.0);
}

TEST_F(DefenderNodesTest, MarkOpponentFicaForaDaAreaComABolaDentroDoGol)
{
  setDefenseBlackboard(-kFieldHalfLength);
  bb_->set("ball", Pose2D{-kFieldHalfLength + 10.0, 0.0, 0.0});
  auto tree = makeTree("<MarkOpponent ball=\"{ball}\" my_goal_x=\"{my_goal_x}\"/>");

  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  const auto x = publishedGoalX();
  ASSERT_TRUE(x.has_value());
  EXPECT_GE(*x, -kFieldHalfLength + kPenaltyDepth + kAreaMargin - 1.0);
}

TEST_F(DefenderNodesTest, SemProfundidadeDaAreaOsNosFalhamEnaoPublicamNada)
{
  setDefenseBlackboard(-kFieldHalfLength, /*with_depth=*/false);
  bb_->set("ball", Pose2D{-2000.0, 0.0, 0.0});
  bb_->set("ball_vel", Vector2D{0.0, 0.0});

  auto shadow = makeTree("<ShadowBall ball=\"{ball}\" ball_vel=\"{ball_vel}\" my_goal_x=\"{my_goal_x}\"/>");
  EXPECT_EQ(shadow.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  auto mark = makeTree("<MarkOpponent ball=\"{ball}\" my_goal_x=\"{my_goal_x}\"/>");
  EXPECT_EQ(mark.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  spinFor(100);
  EXPECT_FALSE(last_goal_.has_value()) << "sem a profundidade da área não pode sair objetivo";
}

TEST_F(DefenderNodesTest, ProfundidadeZeroTambemFalha)
{
  setDefenseBlackboard(-kFieldHalfLength, /*with_depth=*/false);
  bb_->set("penalty_area_depth", 0.0);
  bb_->set("ball", Pose2D{-2000.0, 0.0, 0.0});
  bb_->set("ball_vel", Vector2D{0.0, 0.0});

  auto shadow = makeTree("<ShadowBall ball=\"{ball}\" ball_vel=\"{ball_vel}\" my_goal_x=\"{my_goal_x}\"/>");
  EXPECT_EQ(shadow.tickExactlyOnce(), BT::NodeStatus::FAILURE);
}

// ----------------------------------------------------------------------------------------------
// Posicionamento por ângulo: o defensor fica na linha bola -> centro do gol, não no y da bola
// ----------------------------------------------------------------------------------------------

namespace
{
const char * const kShadowXml =
  "<ShadowBall ball=\"{ball}\" ball_vel=\"{ball_vel}\" my_goal_x=\"{my_goal_x}\"/>";
}  // namespace

TEST_F(DefenderNodesTest, ShadowBallFicaNaLinhaBolaGol)
{
  setDefenseBlackboard(-kFieldHalfLength);
  bb_->set("ball", Pose2D{0.0, 1000.0, 0.0});
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  auto tree = makeTree(kShadowXml);

  const auto goal = tickAndGetGoal(tree);
  ASSERT_TRUE(goal.has_value());
  // Colinear com gol -> bola: produto vetorial (normalizado) quase nulo.
  const double gx = goal->first + kFieldHalfLength;
  const double cross = gx * 1000.0 - goal->second * (0.0 + kFieldHalfLength);
  EXPECT_NEAR(cross / std::hypot(kFieldHalfLength, 1000.0), 0.0, 5.0);
  // Mesmo lado da bola, mas bem menos que o y dela (não "segue a bola no y").
  EXPECT_GT(goal->second, 0.0);
  EXPECT_LT(goal->second, 800.0);
}

TEST_F(DefenderNodesTest, ShadowBallComBolaNoCentroFicaNoEixoDoGol)
{
  setDefenseBlackboard(-kFieldHalfLength);
  bb_->set("ball", Pose2D{0.0, 0.0, 0.0});
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  auto tree = makeTree(kShadowXml);

  const auto goal = tickAndGetGoal(tree);
  ASSERT_TRUE(goal.has_value());
  EXPECT_NEAR(goal->second, 0.0, 1.0);
}

TEST_F(DefenderNodesTest, ShadowBallEspelhaComGolEmXPositivo)
{
  setDefenseBlackboard(kFieldHalfLength);
  bb_->set("ball", Pose2D{0.0, -1000.0, 0.0});
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  auto tree = makeTree(kShadowXml);

  const auto goal = tickAndGetGoal(tree);
  ASSERT_TRUE(goal.has_value());
  EXPECT_GT(goal->first, 0.0);
  EXPECT_LT(goal->second, 0.0);
  EXPECT_NEAR(goal->first, 1125.0, 30.0);  // espelho do caso com o gol em x negativo
}

TEST_F(DefenderNodesTest, ShadowBallNuncaEntraNaAreaEmNenhumaPosicaoDaBola)
{
  setDefenseBlackboard(-kFieldHalfLength);
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  auto tree = makeTree(kShadowXml);

  const double xs[] = {-2300.0, -2100.0, -1500.0, -800.0, 0.0, 1500.0};
  const double ys[] = {-1400.0, -700.0, -100.0, 0.0, 100.0, 700.0, 1400.0};
  for (double bx : xs) {
    for (double by : ys) {
      bb_->set("ball", Pose2D{bx, by, 0.0});
      const auto goal = tickAndGetGoal(tree);
      ASSERT_TRUE(goal.has_value()) << "sem objetivo para a bola em (" << bx << "," << by << ")";
      const bool fora_em_x = (goal->first - (-kFieldHalfLength)) >= kPenaltyDepth + kAreaMargin - 1.0;
      const bool fora_em_y = std::abs(goal->second) >= kPenaltyWidth * 0.5 + kAreaMargin - 1.0;
      EXPECT_TRUE(fora_em_x || fora_em_y)
        << "alvo (" << goal->first << "," << goal->second << ") dentro da area, bola em ("
        << bx << "," << by << ")";
      EXPECT_LE(goal->first, -180.0 + 1.0) << "cruzou o meio-campo";
    }
  }
}

TEST_F(DefenderNodesTest, ShadowBallComBolaColadaNaAreaFechaOAnguloSemTrocarDeLado)
{
  setDefenseBlackboard(-kFieldHalfLength);
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  auto tree = makeTree(kShadowXml);

  // Bola perto do canto da área, em y positivo e depois em y negativo: o defensor fica sempre do
  // MESMO lado da bola e com ângulo menor que o dela (mais perto do eixo do gol).
  for (double by : {800.0, -800.0}) {
    bb_->set("ball", Pose2D{-1900.0, by, 0.0});
    const auto goal = tickAndGetGoal(tree);
    ASSERT_TRUE(goal.has_value());
    EXPECT_GT(goal->second * by, 0.0) << "trocou de lado com a bola em y=" << by;
    const double ball_angle = std::atan2(std::abs(by), -1900.0 + kFieldHalfLength);
    const double def_angle = std::atan2(std::abs(goal->second), goal->first + kFieldHalfLength);
    EXPECT_LT(def_angle, ball_angle);
  }
}

TEST_F(DefenderNodesTest, ShadowBallNuncaTrocaDeLadoQuandoABolaCruzaOEixo)
{
  setDefenseBlackboard(-kFieldHalfLength);
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  auto tree = makeTree(kShadowXml);

  // Varre a bola de y=-1000 a y=+1000 em várias distâncias: o defensor acompanha o sinal da bola e
  // nunca fica em y oposto (com folga de 1 mm para o caso y=0).
  for (double bx : {-1500.0, -800.0, 0.0}) {
    for (double by = -1000.0; by <= 1000.0; by += 250.0) {
      bb_->set("ball", Pose2D{bx, by, 0.0});
      const auto goal = tickAndGetGoal(tree);
      ASSERT_TRUE(goal.has_value());
      EXPECT_GE(goal->second * by, -1.0)
        << "lado oposto: bola (" << bx << "," << by << ") defensor y=" << goal->second;
    }
  }
}

TEST_F(DefenderNodesTest, ShadowBallIgnoraAVelocidadeDaBolaQuandoElaSaiDoNossoGol)
{
  setDefenseBlackboard(-kFieldHalfLength);
  auto tree = makeTree(kShadowXml);

  // Nosso chute para o campo adversário (vx > 0 com o gol em x negativo): não é para perseguir.
  bb_->set("ball", Pose2D{-500.0, 600.0, 0.0});
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  const auto parada = tickAndGetGoal(tree);
  bb_->set("ball_vel", Vector2D{3000.0, 1500.0});
  const auto saindo = tickAndGetGoal(tree);

  ASSERT_TRUE(parada.has_value());
  ASSERT_TRUE(saindo.has_value());
  EXPECT_NEAR(parada->first, saindo->first, 1.0);
  EXPECT_NEAR(parada->second, saindo->second, 1.0);
}

TEST_F(DefenderNodesTest, ShadowBallAntecipaABolaQuandoElaVemParaONossoGol)
{
  setDefenseBlackboard(-kFieldHalfLength);
  auto tree = makeTree(kShadowXml);

  bb_->set("ball", Pose2D{0.0, 0.0, 0.0});
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  const auto parada = tickAndGetGoal(tree);
  // Chute adversário em direção ao nosso gol e para o lado: o defensor já vai para a trajetória.
  bb_->set("ball_vel", Vector2D{-2500.0, 2000.0});
  const auto vindo = tickAndGetGoal(tree);

  ASSERT_TRUE(parada.has_value());
  ASSERT_TRUE(vindo.has_value());
  EXPECT_GT(vindo->second, parada->second + 100.0);
}

TEST_F(DefenderNodesTest, ShadowBallComABolaPertoMasForaDoAnelFicaNaLinhaBolaGol)
{
  setDefenseBlackboard(-kFieldHalfLength);
  bb_->set("ball", Pose2D{-800.0, 600.0, 0.0});   // a ~500 mm do alvo: perto, mas não colada
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  auto tree = makeTree(kShadowXml);

  const auto goal = tickAndGetGoal(tree);
  ASSERT_TRUE(goal.has_value());
  // Distância do alvo à reta bola -> centro do gol: o defensor está EM FRENTE da bola, não ao lado.
  const double gx = -kFieldHalfLength;
  const double bx = -800.0 - gx;
  const double by = 600.0;
  const double cross = (goal->first - gx) * by - goal->second * bx;
  EXPECT_NEAR(cross / std::hypot(bx, by), 0.0, 10.0);
  // E entre a bola e o gol.
  EXPECT_LT(goal->first, -800.0);
}

TEST_F(DefenderNodesTest, MarkOpponentFicaA400mmDaBolaNaLinhaBolaGol)
{
  setDefenseBlackboard(-kFieldHalfLength);
  bb_->set("ball", Pose2D{-800.0, 600.0, 0.0});
  auto tree = makeTree("<MarkOpponent ball=\"{ball}\" my_goal_x=\"{my_goal_x}\"/>");

  const auto goal = tickAndGetGoal(tree);
  ASSERT_TRUE(goal.has_value());
  EXPECT_NEAR(std::hypot(goal->first + 800.0, goal->second - 600.0), 400.0, 5.0);
  const double gx = -kFieldHalfLength;
  const double cross = (goal->first - gx) * 600.0 - goal->second * (-800.0 - gx);
  EXPECT_NEAR(cross / std::hypot(-800.0 - gx, 600.0), 0.0, 10.0);
}

TEST_F(DefenderNodesTest, ShadowBallComABolaQuePassouDoDefensorEncostaNaBordaDaAreaSemFicarAlemDela)
{
  setDefenseBlackboard(-kFieldHalfLength);
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  auto tree = makeTree(kShadowXml);

  // A bola já passou do raio em que o defensor costuma ficar e está colada à área: ele vai para a
  // borda da área (do lado do gol em relação à bola), e não para um ponto ALÉM da bola, parado
  // olhando para ela.
  bb_->set("ball", Pose2D{-1500.0, 700.0, 0.0});
  const auto goal = tickAndGetGoal(tree);
  ASSERT_TRUE(goal.has_value());
  EXPECT_LE(goal->first, -1500.0 + 1.0) << "o defensor ficou além da bola";
  EXPECT_GE(goal->first, -kFieldHalfLength + kPenaltyDepth + kAreaMargin - 1.0);
}

TEST_F(DefenderNodesTest, ShadowBallSemLarguraDaAreaFalha)
{
  setDefenseBlackboard(-kFieldHalfLength);
  bb_->set("penalty_area_width", 0.0);
  bb_->set("ball", Pose2D{0.0, 0.0, 0.0});
  bb_->set("ball_vel", Vector2D{0.0, 0.0});
  auto tree = makeTree(kShadowXml);

  last_goal_.reset();
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  spinFor(100);
  EXPECT_FALSE(last_goal_.has_value());
}

// ----------------------------------------------------------------------------------------------
// SC-15: a troca de papel acontece em um tick
// ----------------------------------------------------------------------------------------------

TEST_F(DefenderNodesTest, AtacanteViraDefensorNoMesmoTick)
{
  auto tree = makeTree(
    "<ReactiveFallback>"
    "  <ReactiveSequence name=\"AttackerRole\"><HasRole role=\"attacker\"/><FakeAttack/></ReactiveSequence>"
    "  <ReactiveSequence name=\"DefenderRole\"><HasRole role=\"defender\"/><FakeDefend/></ReactiveSequence>"
    "</ReactiveFallback>");

  bb_->set("role", std::string("attacker"));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(FakeDefend::ticks, 0);

  // Perda de posse: o atribuidor troca os papéis.
  bb_->set("role", std::string("defender"));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(FakeAttack::halts, 1) << "o ramo do atacante deve ser interrompido nesse tick";
  EXPECT_EQ(FakeDefend::ticks, 1) << "o defensor deve assumir nesse mesmo tick";
}

TEST_F(DefenderNodesTest, RoleOciosoNaoAtivaNenhumRamo)
{
  auto tree = makeTree(
    "<ReactiveFallback>"
    "  <ReactiveSequence><HasRole role=\"attacker\"/><FakeAttack/></ReactiveSequence>"
    "  <ReactiveSequence><HasRole role=\"defender\"/><FakeDefend/></ReactiveSequence>"
    "</ReactiveFallback>");

  bb_->set("role", std::string("idle"));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(FakeDefend::ticks, 0);
}

}  // namespace
