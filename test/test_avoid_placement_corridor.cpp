#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "oxebots_strategy/avoid_placement_corridor_node.hpp"
#include "oxebots_strategy/hold_position_node.hpp"
#include "oxebots_strategy/set_motion_limit_node.hpp"

namespace
{

using namespace std::chrono_literals;
using oxebots_interfaces::msg::RobotGoal;

void setCorridorInputs(const BT::Blackboard::Ptr & blackboard, bool has_designated = true)
{
  blackboard->set("robot_id", 1u);
  blackboard->set("robot_x", 0.0);
  blackboard->set("robot_y", 0.0);
  blackboard->set("robot_yaw", 0.25);
  blackboard->set("ball_x", 0.0);
  blackboard->set("ball_y", 0.0);
  blackboard->set("designated_x", 1000.0);
  blackboard->set("designated_y", 0.0);
  blackboard->set("has_designated", has_designated);
}

BT::BehaviorTreeFactory makeFactory(const rclcpp::Node::SharedPtr & node)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<oxebots_strategy::AvoidPlacementCorridorNode>(
    "AvoidPlacementCorridor", node);
  factory.registerNodeType<oxebots_strategy::HoldPositionNode>("HoldPosition", node);
  factory.registerNodeType<oxebots_strategy::SetMotionLimitNode>("SetMotionLimit", node);
  return factory;
}

bool spinUntil(
  const rclcpp::Node::SharedPtr & node,
  const std::function<bool()> & condition)
{
  for (int attempt = 0; attempt < 50; ++attempt) {
    rclcpp::spin_some(node);
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

class AvoidPlacementCorridorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("avoid_placement_corridor_test");
  }

  void TearDown() override
  {
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
};

}  // namespace

TEST_F(AvoidPlacementCorridorTest, InsideCorridorPublishesEscapeGoal)
{
  auto factory = makeFactory(node_);
  auto blackboard = BT::Blackboard::create();
  setCorridorInputs(blackboard);

  RobotGoal::SharedPtr received_goal;
  auto goal_sub = node_->create_subscription<RobotGoal>(
    "/robot_goal", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&received_goal](const RobotGoal::SharedPtr msg) {received_goal = msg;});
  ASSERT_TRUE(goal_sub);

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <AvoidPlacementCorridor margin_mm="550.0" max_speed="0.75"/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(spinUntil(node_, [&received_goal]() {return received_goal != nullptr;}));
  ASSERT_EQ(received_goal->robot_id, 1);
  EXPECT_FLOAT_EQ(received_goal->max_speed, 0.75f);
  EXPECT_EQ(
    received_goal->planner_type,
    oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE);

  const double target_x = received_goal->pose.pose.position.x * 1000.0;
  const double target_y = received_goal->pose.pose.position.y * 1000.0;
  EXPECT_GE(std::hypot(target_x, target_y), 550.0);
}

TEST_F(AvoidPlacementCorridorTest, OutsideCorridorReturnsSuccess)
{
  auto factory = makeFactory(node_);
  auto blackboard = BT::Blackboard::create();
  setCorridorInputs(blackboard);
  blackboard->set("robot_y", 600.0);

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <AvoidPlacementCorridor/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(AvoidPlacementCorridorTest, MissingDesignatedPositionReturnsFailure)
{
  auto factory = makeFactory(node_);
  auto blackboard = BT::Blackboard::create();
  setCorridorInputs(blackboard, false);

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="Test">
           <AvoidPlacementCorridor/>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(AvoidPlacementCorridorTest, SC11BallPlacementLimitsSpeedAndLeavesCorridor)
{
  auto factory = makeFactory(node_);
  auto blackboard = BT::Blackboard::create();
  setCorridorInputs(blackboard);

  std_msgs::msg::Float32::SharedPtr received_limit;
  RobotGoal::SharedPtr received_goal;
  auto limit_sub = node_->create_subscription<std_msgs::msg::Float32>(
    "/robot_motion_limit", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&received_limit](const std_msgs::msg::Float32::SharedPtr msg) {received_limit = msg;});
  auto goal_sub = node_->create_subscription<RobotGoal>(
    "/robot_goal", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&received_goal](const RobotGoal::SharedPtr msg) {received_goal = msg;});
  ASSERT_TRUE(limit_sub);
  ASSERT_TRUE(goal_sub);

  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="BallPlacementTheirTree">
           <ReactiveSequence>
             <SetMotionLimit limit_mps="0.75"/>
             <ReactiveSequence>
               <AvoidPlacementCorridor margin_mm="550.0" max_speed="0.75"/>
               <HoldPosition/>
             </ReactiveSequence>
           </ReactiveSequence>
         </BehaviorTree>
       </root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(spinUntil(node_, [&received_limit, &received_goal]() {
    return received_limit != nullptr && received_goal != nullptr;
  }));
  EXPECT_FLOAT_EQ(received_limit->data, 0.75f);
  EXPECT_EQ(received_goal->robot_id, 1);
  EXPECT_FLOAT_EQ(received_goal->max_speed, 0.75f);

  const double target_x = received_goal->pose.pose.position.x * 1000.0;
  const double target_y = received_goal->pose.pose.position.y * 1000.0;
  EXPECT_GE(std::hypot(target_x, target_y), 550.0);
}
