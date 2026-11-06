#include "oxebots_strategy/node_registry.hpp"

#include <memory>
#include <string>

#include "behaviortree_cpp/xml_parsing.h"
#include "oxebots_strategy/aim_at.hpp"
#include "oxebots_strategy/align_to_ball_node.h"
#include "oxebots_strategy/calculate_interception_node.h"
#include "oxebots_strategy/calculate_kick_off_pose.hpp"
#include "oxebots_strategy/compute_legal_slot_node.hpp"
#include "oxebots_strategy/defender_nodes.hpp"
#include "oxebots_strategy/game_conditions.hpp"
#include "oxebots_strategy/go_to_point_node.h"
#include "oxebots_strategy/goal_line_defend_node.h"
#include "oxebots_strategy/goalkeeper_node.h"
#include "oxebots_strategy/has_line_of_sight.hpp"
#include "oxebots_strategy/hold_position_node.hpp"
#include "oxebots_strategy/is_ball_close_condition.h"
#include "oxebots_strategy/is_ball_closest_condition.h"
#include "oxebots_strategy/is_ball_in_area_condition.h"
#include "oxebots_strategy/is_ball_in_our_half_condition.h"
#include "oxebots_strategy/keep_distance.hpp"
#include "oxebots_strategy/kick_ball_node.h"
#include "oxebots_strategy/kick_once_node.hpp"
#include "oxebots_strategy/lateral_clear_node.h"
#include "oxebots_strategy/position_bisector_node.h"
#include "oxebots_strategy/precision_kick_node.h"
#include "oxebots_strategy/set_motion_limit_node.hpp"
#include "oxebots_strategy/smother_save_node.h"
#include "oxebots_strategy/update_ball_position_node.h"
#include "oxebots_strategy/wait_ball_in_play_condition.hpp"

namespace oxebots_strategy
{

namespace
{

BT::Blackboard * resolveBlackboard(
  const BT::TreeNode & tree_node,
  const BT::Blackboard::Ptr & fallback_bb)
{
  if (fallback_bb) {
    return fallback_bb.get();
  }
  if (tree_node.config().blackboard) {
    return tree_node.config().blackboard->rootBlackboard();
  }
  return nullptr;
}

}  // namespace

void registerStrategyNodes(
  BT::BehaviorTreeFactory & factory,
  const rclcpp::Node::SharedPtr & node,
  const BT::Blackboard::Ptr & blackboard,
  const rclcpp::Publisher<oxebots_interfaces::msg::RobotMotionOverride>::SharedPtr & override_pub,
  int robot_id)
{
  if (node) {
    configureDefenderController(node, static_cast<uint32_t>(robot_id));
  }

  factory.registerNodeType<oxebots_strategy::GoToPointNode>("GoToPoint", node);
  factory.registerNodeType<oxebots_strategy::KickBallNode>("KickBall", node);
  factory.registerNodeType<oxebots_strategy::AlignToBallNode>("AlignToBall", node);
  factory.registerNodeType<oxebots_strategy::UpdateBallPositionNode>("UpdateBallPosition", node);
  factory.registerNodeType<oxebots_strategy::CalculateInterceptionNode>(
    "CalculateInterception", node);
  factory.registerNodeType<oxebots_strategy::IsBallCloseCondition>("IsBallClose", node);
  factory.registerNodeType<oxebots_strategy::GoalkeeperNode>("Goalkeeper", node);
  factory.registerNodeType<oxebots_strategy::PrecisionKickNode>("PrecisionKick", node);
  factory.registerNodeType<oxebots_strategy::GoalLineDefendNode>("GoalLineDefend", node);
  factory.registerNodeType<oxebots_strategy::IsBallInAreaCondition>("IsBallInArea", node);
  factory.registerNodeType<oxebots_strategy::IsBallClosestCondition>("IsBallClosest", node);
  factory.registerNodeType<oxebots_strategy::IsBallInOurHalfCondition>("IsBallInOurHalf", node);
  factory.registerNodeType<oxebots_strategy::LateralClearNode>("LateralClear", node);
  factory.registerNodeType<oxebots_strategy::PositionBisectorNode>("PositionBisector", node);
  factory.registerNodeType<oxebots_strategy::SmotherSaveNode>("SmotherSave", node);
  factory.registerNodeType<oxebots_strategy::CalculateKickOffPose>("CalculateKickOffPose");
  factory.registerNodeType<oxebots_strategy::KeepDistance>("KeepDistance");
  factory.registerNodeType<oxebots_strategy::AimAt>("AimAt");
  factory.registerNodeType<oxebots_strategy::HasLineOfSight>("HasLineOfSight");
  factory.registerNodeType<oxebots_strategy::CheckGameStateCondition>("CheckGameState");
  factory.registerNodeType<oxebots_strategy::WeAreFavoredCondition>("WeAreFavored");
  factory.registerNodeType<oxebots_strategy::IsDataFreshCondition>("IsDataFresh");
  factory.registerNodeType<oxebots_strategy::HasRoleCondition>("HasRole");
  factory.registerNodeType<oxebots_strategy::HoldPositionNode>("HoldPosition", node);
  factory.registerNodeType<oxebots_strategy::SetMotionLimitNode>("SetMotionLimit", node);
  factory.registerNodeType<oxebots_strategy::ComputeLegalSlotNode>("ComputeLegalSlot", node);
  factory.registerNodeType<oxebots_strategy::KickOnceNode>("KickOnce", node);
  factory.registerNodeType<oxebots_strategy::WaitBallInPlayCondition>("WaitBallInPlay");

  // Nós da estratégia defensiva (defender_tree.xml)
  factory.registerNodeType<IsBallInOpponentField>("IsBallInOpponentField");
  factory.registerNodeType<IsOpponentControllingBall>("IsOpponentControllingBall");
  factory.registerNodeType<ShadowBall>("ShadowBall");
  factory.registerNodeType<MarkOpponent>("MarkOpponent");
  factory.registerNodeType<GoToClamped>("GoToClamped");

  factory.registerSimpleCondition(
    "IsValueTrue",
    [](BT::TreeNode & tree_node) {
      bool val = false;
      if (tree_node.getInput("value", val)) {
        return val ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::FAILURE;
    },
    {BT::InputPort<bool>("value", "Valor booleano a avaliar")});

  factory.registerSimpleCondition(
    "IsThisNodeID",
    [robot_id](BT::TreeNode & tree_node) {
      unsigned int target_id = 0;
      if (!tree_node.getInput("id", target_id)) {
        return BT::NodeStatus::FAILURE;
      }
      return (static_cast<unsigned int>(robot_id) == target_id)
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
    },
    {BT::InputPort<unsigned int>("id", "ID esperado do processo strategy_node")});

  factory.registerSimpleCondition(
    "IsMyRole",
    [blackboard](BT::TreeNode & tree_node) {
      std::string role;
      uint32_t my_id = 0;
      uint32_t target_id = 0;
      if (!tree_node.getInput("role", role)) {
        return BT::NodeStatus::FAILURE;
      }
      BT::Blackboard * bb = resolveBlackboard(tree_node, blackboard);
      if (!bb || !bb->get("robot_id", my_id)) {
        return BT::NodeStatus::FAILURE;
      }
      if (role == "goalkeeper") {
        if (bb->get("goalkeeper_id", target_id)) {
          return (my_id == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
      } else if (role == "attacker") {
        if (bb->get("attacker_id", target_id)) {
          return (my_id == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
      } else if (role == "defender") {
        if (bb->get("defender_id", target_id)) {
          return (my_id == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
      } else if (role == "kicker") {
        if (bb->get("kicker_id", target_id)) {
          return (my_id == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
      }
      return BT::NodeStatus::FAILURE;
    },
    {BT::InputPort<std::string>("role", "Papel esperado (attacker | defender)")});

  factory.registerSimpleCondition(
    "CheckGCCommand",
    [blackboard](BT::TreeNode & tree_node) {
      int expected = -1;
      if (!tree_node.getInput("expected", expected)) {
        return BT::NodeStatus::FAILURE;
      }
      int current = -1;
      BT::Blackboard * bb = resolveBlackboard(tree_node, blackboard);
      if (!bb || !bb->get("gc_command", current)) {
        return BT::NodeStatus::FAILURE;
      }
      return (current == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    },
    {BT::InputPort<int>("expected", "Código inteiro do comando bruto do Game Controller")});

  factory.registerSimpleCondition(
    "IsRobotId",
    [blackboard](BT::TreeNode & tree_node) {
      uint32_t target_id = 0;
      uint32_t my_id = 0;
      if (!tree_node.getInput("robot_id", target_id)) {
        return BT::NodeStatus::FAILURE;
      }
      BT::Blackboard * bb = resolveBlackboard(tree_node, blackboard);
      if (!bb || !bb->get("robot_id", my_id)) {
        return BT::NodeStatus::FAILURE;
      }
      return (my_id == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    },
    {BT::InputPort<uint32_t>("robot_id", "ID esperado do robô")});

  factory.registerSimpleAction(
    "Halt",
    [node, override_pub, robot_id](BT::TreeNode & tree_node) {
      std::string reason = "pelo GC";
      std::string input_reason;
      if (tree_node.getInput("reason", input_reason)) {
        if (input_reason == "idle") {
          reason = "por estar ocioso (sem papel)";
        } else if (input_reason == "gc_halt") {
          reason = "por HALT do arbitro";
        } else if (input_reason == "timeout") {
          reason = "por TIMEOUT";
        } else if (input_reason == "stale_data") {
          reason = "por dados desatualizados (stale/modo degradado)";
        } else {
          reason = "motivo: " + input_reason;
        }
      }
      if (node) {
        RCLCPP_INFO_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "Robô %d parado %s", robot_id, reason.c_str());
      }

      if (override_pub) {
        auto override_msg = std::make_unique<oxebots_interfaces::msg::RobotMotionOverride>();
        override_msg->robot_id = robot_id;
        override_msg->mode = oxebots_interfaces::msg::RobotMotionOverride::MODE_HALT;
        override_pub->publish(std::move(override_msg));
      }
      return BT::NodeStatus::SUCCESS;
    },
    {BT::InputPort<std::string>("reason", "idle", "Opcional: motivo da parada (gc_halt, timeout, stale_data, idle)")});
}

std::string generateTreeNodesModelXML(
  const BT::BehaviorTreeFactory & factory,
  bool include_builtin)
{
  return BT::writeTreeNodesModelXML(factory, include_builtin);
}

}  // namespace oxebots_strategy
