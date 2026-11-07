#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/robot_motion_override.hpp"
#include "oxebots_strategy/ball_held_timer_condition.hpp"
#include "oxebots_strategy/goal_line_defend_node.h"
#include "oxebots_strategy/goalkeeper_node.h"
#include "oxebots_strategy/lateral_clear_node.h"
#include "oxebots_strategy/penalty_keeper_line_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace oxebots_strategy
{

class GoalkeeperK01K02Test : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_goalkeeper_k01_k02_node");

    auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
    goal_sub_ = node_->create_subscription<oxebots_interfaces::msg::RobotGoal>(
      "/robot_goal", goal_qos,
      [this](const oxebots_interfaces::msg::RobotGoal::SharedPtr msg) {
        last_goal_ = *msg;
      });

    override_sub_ = node_->create_subscription<oxebots_interfaces::msg::RobotMotionOverride>(
      "/robot_motion_override", 10,
      [this](const oxebots_interfaces::msg::RobotMotionOverride::SharedPtr msg) {
        last_override_ = *msg;
      });

    factory_.registerNodeType<GoalkeeperNode>("Goalkeeper", node_);
    factory_.registerNodeType<PenaltyKeeperLineNode>("PenaltyKeeperLine", node_);
    factory_.registerNodeType<BallHeldTimerCondition>("BallHeldTimer");
    factory_.registerNodeType<GoalLineDefendNode>("GoalLineDefend", node_);
    factory_.registerNodeType<LateralClearNode>("LateralClear", node_);

    bb_ = BT::Blackboard::create();
  }

  void populateStandardGeometry(double my_goal_x = -4500.0)
  {
    bb_->set("my_goal_x", my_goal_x);
    bb_->set("own_goal_x", my_goal_x);
    bb_->set("field_width", 6000.0);
    bb_->set("penalty_area_depth", 1000.0);
    bb_->set("penalty_area_width", 2000.0);
    bb_->set("goal_width", 1000.0);
  }

  void populatePoseAndBall(
    uint32_t robot_id, double rx, double ry, double ryaw,
    double bx, double by)
  {
    bb_->set("robot_id", robot_id);
    bb_->set("goalkeeper_id", robot_id);
    bb_->set("is_goalkeeper", true);
    bb_->set("robot_x", rx);
    bb_->set("robot_y", ry);
    bb_->set("robot_yaw", ryaw);
    bb_->set("ball_x", bx);
    bb_->set("ball_y", by);
    bb_->set("ball_vx", 0.0);
    bb_->set("ball_vy", 0.0);
    bb_->set("pose_age_ms", 10.0);
    bb_->set("ball_age_ms", 10.0);
  }

  void spinBriefly()
  {
    rclcpp::spin_some(node_);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::RobotMotionOverride>::SharedPtr override_sub_;
  std::optional<oxebots_interfaces::msg::RobotGoal> last_goal_;
  std::optional<oxebots_interfaces::msg::RobotMotionOverride> last_override_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr bb_;
};

// K01 / SC-17: Identidade dinâmica do goleiro (sem robot_id == 0 fixo) e falha se não for goleiro
TEST_F(GoalkeeperK01K02Test, K01_DynamicGoalkeeperIdAndRoleEnforcement)
{
  populateStandardGeometry(-4500.0);
  populatePoseAndBall(3u, -4400.0, 0.0, 0.0, 0.0, 200.0);

  auto tree = factory_.createTreeFromText(
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <Goalkeeper robot_id="{robot_id}"/>
      </BehaviorTree>
    </root>
    )",
    bb_);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  spinBriefly();
  ASSERT_TRUE(last_goal_.has_value());
  EXPECT_EQ(last_goal_->robot_id, 3u);
  EXPECT_NEAR(last_goal_->pose.pose.position.x * 1000.0, -4380.0, 1.0);
  EXPECT_NEAR(last_goal_->pose.pose.position.y * 1000.0, 200.0, 1.0);

  // Se o robô perde o papel de goleiro (ex.: GC trocou goleiro para 5 ou goleiro ausente = 999)
  bb_->set("is_goalkeeper", false);
  bb_->set("goalkeeper_id", 999u);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// K01: Goalkeeper, GoalLineDefend e LateralClear devolvem FAILURE sem dados/geometria ou com dados expirados
TEST_F(GoalkeeperK01K02Test, K01_FailureWhenMissingGeometryOrDataOrStale)
{
  auto gk_tree = factory_.createTreeFromText(
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <Goalkeeper robot_id="{robot_id}"/>
      </BehaviorTree>
    </root>
    )",
    bb_);

  // 1. Sem geometria nem dados -> FAILURE
  bb_->set("robot_id", 1u);
  EXPECT_EQ(gk_tree.tickOnce(), BT::NodeStatus::FAILURE);

  // 2. Com geometria, mas sem pose/bola -> FAILURE
  populateStandardGeometry(-4500.0);
  EXPECT_EQ(gk_tree.tickOnce(), BT::NodeStatus::FAILURE);

  // 3. Com geometria e pose/bola válidas -> RUNNING
  populatePoseAndBall(1u, -4400.0, 0.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(gk_tree.tickOnce(), BT::NodeStatus::RUNNING);

  // 4. Com pose_age_ms > 500 ms -> FAILURE
  bb_->set("pose_age_ms", 650.0);
  EXPECT_EQ(gk_tree.tickOnce(), BT::NodeStatus::FAILURE);

  // 5. GoalLineDefend e LateralClear sem geometria -> FAILURE
  auto empty_bb = BT::Blackboard::create();
  empty_bb->set("robot_id", 0u);
  empty_bb->set("ball_x", 0.0);
  empty_bb->set("ball_y", 0.0);

  auto gld_tree = factory_.createTreeFromText(
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <GoalLineDefend robot_id="{robot_id}" ball_x="{ball_x}" ball_y="{ball_y}"/>
      </BehaviorTree>
    </root>
    )",
    empty_bb);
  EXPECT_EQ(gld_tree.tickOnce(), BT::NodeStatus::FAILURE);

  auto lc_tree = factory_.createTreeFromText(
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <LateralClear robot_id="{robot_id}" ball_x="{ball_x}" ball_y="{ball_y}" kick_speed="{ks}"/>
      </BehaviorTree>
    </root>
    )",
    empty_bb);
  EXPECT_EQ(lc_tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// K01 / SC-25: Limite de posse na área (max_hold_s) força afastamento urgente antes dos 10 s
TEST_F(GoalkeeperK01K02Test, K01_HoldLimitForcesUrgentClearanceInArea)
{
  populateStandardGeometry(-4500.0);
  // Goleiro entre o gol (-4500) e a bola (-4000, 100), próximo da bola (150 mm),
  // mas ainda não alinhado na direção lateral normal (onde sem urgência iria ao pré-chute com D*)
  populatePoseAndBall(2u, -4150.0, 100.0, 0.0, -4000.0, 100.0);

  auto tree = factory_.createTreeFromText(
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <Goalkeeper robot_id="{robot_id}" max_hold_s="0.05" kick_speed="2.8"/>
      </BehaviorTree>
    </root>
    )",
    bb_);

  // Antes de estourar max_hold_s: usa D* para ir ao ponto de pré-chute atrás da bola
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  spinBriefly();
  ASSERT_TRUE(last_goal_.has_value());
  EXPECT_EQ(
    last_goal_->planner_type,
    oxebots_interfaces::msg::RobotGoal::PLANNER_DSTAR);
  EXPECT_FALSE(last_override_.has_value());

  // Após max_hold_s (50 ms): entra em limpeza urgente (linha reta direto na bola + chute)
  std::this_thread::sleep_for(std::chrono::milliseconds(70));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  spinBriefly();
  ASSERT_TRUE(last_goal_.has_value());
  EXPECT_EQ(
    last_goal_->planner_type,
    oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE);
  ASSERT_TRUE(last_override_.has_value());
  EXPECT_EQ(last_override_->robot_id, 2u);
  EXPECT_EQ(
    last_override_->mode,
    oxebots_interfaces::msg::RobotMotionOverride::MODE_KICK);
  EXPECT_NEAR(last_override_->kick_speed, 2.8, 1e-3);
}

// K01 / SC-25: Condição BallHeldTimer falha após max_s com a bola parada na área própria
TEST_F(GoalkeeperK01K02Test, K01_BallHeldTimerConditionDetectsStagnantBallInArea)
{
  populateStandardGeometry(-4500.0);
  // Bola fora da área (-2000, 0) -> SUCCESS
  populatePoseAndBall(0u, -4400.0, 0.0, 0.0, -2000.0, 0.0);

  auto tree = factory_.createTreeFromText(
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <BallHeldTimer max_s="0.05" area="own_defense"/>
      </BehaviorTree>
    </root>
    )",
    bb_);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // Bola entra na área de defesa (-4000, 0) e fica parada: início -> SUCCESS
  bb_->set("ball_x", -4000.0);
  bb_->set("ball_y", 0.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // Passados >= 50 ms parada na área -> FAILURE (gatilho de limpeza urgente)
  std::this_thread::sleep_for(std::chrono::milliseconds(70));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Se a bola se move significativamente (>= 80 mm) dentro da área, o timer reinicia -> SUCCESS
  bb_->set("ball_x", -3850.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// K02 / SC-10: PenaltyKeeperLine mantém o goleiro cravado a 90 mm da linha de gol mesmo com bola na área
TEST_F(GoalkeeperK01K02Test, K02_PenaltyKeeperLineStaysOnGoalLineAt90mm)
{
  populateStandardGeometry(-4500.0);
  // Bola dentro da própria área (-3800, 250) durante pênalti adversário
  populatePoseAndBall(4u, -4400.0, 0.0, 0.0, -3800.0, 250.0);

  auto tree = factory_.createTreeFromText(
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <PenaltyKeeperLine side="defend"/>
      </BehaviorTree>
    </root>
    )",
    bb_);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  spinBriefly();
  ASSERT_TRUE(last_goal_.has_value());
  EXPECT_EQ(last_goal_->robot_id, 4u);
  // -4500 + 90 = -4410 mm (-4.41 m)
  EXPECT_NEAR(last_goal_->pose.pose.position.x * 1000.0, -4410.0, 1.0);
  EXPECT_NEAR(last_goal_->pose.pose.position.y * 1000.0, 250.0, 1.0);
  EXPECT_TRUE(last_goal_->disable_arrival_latch);
  EXPECT_EQ(
    last_goal_->planner_type,
    oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE);

  // Testando no lado positivo (+4500) e com bola além do limite Y da trave (goal_width=1000 -> limit_y = 410)
  populateStandardGeometry(4500.0);
  bb_->set("ball_x", 3800.0);
  bb_->set("ball_y", -900.0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  spinBriefly();
  ASSERT_TRUE(last_goal_.has_value());
  // +4500 - 90 = +4410 mm
  EXPECT_NEAR(last_goal_->pose.pose.position.x * 1000.0, 4410.0, 1.0);
  EXPECT_NEAR(last_goal_->pose.pose.position.y * 1000.0, -410.0, 1.0);
}

// K02 / SC-10: Goalkeeper suporta mode="penalty_def" e mode="hold"
TEST_F(GoalkeeperK01K02Test, K02_GoalkeeperModesPenaltyDefAndHold)
{
  populateStandardGeometry(-4500.0);
  // Bola dentro da área bem na frente do goleiro (-3900, 150)
  populatePoseAndBall(1u, -4250.0, -120.0, 0.25, -3900.0, 150.0);

  // 1. mode="penalty_def": vai para a linha de gol (-4410, 150) e NÃO sai na bola
  auto penalty_tree = factory_.createTreeFromText(
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <Goalkeeper robot_id="{robot_id}" mode="penalty_def"/>
      </BehaviorTree>
    </root>
    )",
    bb_);

  EXPECT_EQ(penalty_tree.tickOnce(), BT::NodeStatus::RUNNING);
  spinBriefly();
  ASSERT_TRUE(last_goal_.has_value());
  EXPECT_NEAR(last_goal_->pose.pose.position.x * 1000.0, -4410.0, 1.0);
  EXPECT_NEAR(last_goal_->pose.pose.position.y * 1000.0, 150.0, 1.0);
  EXPECT_TRUE(last_goal_->disable_arrival_latch);
  EXPECT_FALSE(last_override_.has_value());

  // 2. mode="hold": mantém a pose atual do goleiro (-4250, -120, yaw=0.25) com disable_arrival_latch
  auto hold_tree = factory_.createTreeFromText(
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <Goalkeeper robot_id="{robot_id}" mode="hold"/>
      </BehaviorTree>
    </root>
    )",
    bb_);

  EXPECT_EQ(hold_tree.tickOnce(), BT::NodeStatus::RUNNING);
  spinBriefly();
  ASSERT_TRUE(last_goal_.has_value());
  EXPECT_NEAR(last_goal_->pose.pose.position.x * 1000.0, -4250.0, 1.0);
  EXPECT_NEAR(last_goal_->pose.pose.position.y * 1000.0, -120.0, 1.0);
  EXPECT_TRUE(last_goal_->disable_arrival_latch);
}

}  // namespace oxebots_strategy
