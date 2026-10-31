// Despeja o TreeNodesModel (portas e padrões de todas as folhas) para o validate_bt.py.
//
// Os registros abaixo espelham os de StrategyNode::init() em strategy_node.cpp. Os nós só são
// registrados (o manifesto vem de providedPorts()); nenhum é instanciado, então não há robô,
// tópico nem blackboard envolvidos. Se uma folha for registrada no strategy_node.cpp e não
// aqui, o validate_bt.py avisa (regra R8).
#include <iostream>
#include <memory>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "oxebots_strategy/aim_at.hpp"
#include "oxebots_strategy/align_to_ball_node.h"
#include "oxebots_strategy/calculate_interception_node.h"
#include "oxebots_strategy/calculate_kick_off_pose.hpp"
#include "oxebots_strategy/defender_nodes.hpp"
#include "oxebots_strategy/go_to_point_node.h"
#include "oxebots_strategy/goal_line_defend_node.h"
#include "oxebots_strategy/goalkeeper_node.h"
#include "oxebots_strategy/has_line_of_sight.hpp"
#include "oxebots_strategy/is_ball_close_condition.h"
#include "oxebots_strategy/is_ball_closest_condition.h"
#include "oxebots_strategy/is_ball_in_area_condition.h"
#include "oxebots_strategy/is_ball_in_our_half_condition.h"
#include "oxebots_strategy/keep_distance.hpp"
#include "oxebots_strategy/kick_ball_node.h"
#include "oxebots_strategy/lateral_clear_node.h"
#include "oxebots_strategy/position_bisector_node.h"
#include "oxebots_strategy/precision_kick_node.h"
#include "oxebots_strategy/smother_save_node.h"
#include "oxebots_strategy/update_ball_position_node.h"
#include "rclcpp/rclcpp.hpp"

namespace
{

// Corpo irrelevante: só as portas importam para o manifesto.
BT::NodeStatus noop(BT::TreeNode &)
{
  return BT::NodeStatus::SUCCESS;
}

void register_all(BT::BehaviorTreeFactory & factory, const rclcpp::Node::SharedPtr & node)
{
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

  factory.registerNodeType<IsBallInOpponentField>("IsBallInOpponentField");
  factory.registerNodeType<IsOpponentControllingBall>("IsOpponentControllingBall");
  factory.registerNodeType<ShadowBall>("ShadowBall");
  factory.registerNodeType<MarkOpponent>("MarkOpponent");
  factory.registerNodeType<GoToClamped>("GoToClamped");

  // Nós registrados por lambda no strategy_node.cpp: aqui só as portas.
  factory.registerSimpleCondition("IsValueTrue", noop, {BT::InputPort<bool>("value")});
  factory.registerSimpleCondition("IsThisNodeID", noop, {BT::InputPort<unsigned int>("id")});
  factory.registerSimpleCondition("IsMyRole", noop, {BT::InputPort<std::string>("role")});
  factory.registerSimpleCondition("CheckGCCommand", noop, {BT::InputPort<int>("expected")});
  factory.registerSimpleCondition("IsRobotId", noop, {BT::InputPort<uint32_t>("robot_id")});
  factory.registerSimpleAction("Halt", noop, {BT::InputPort<std::string>("reason")});
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dump_bt_model");

  BT::BehaviorTreeFactory factory;
  register_all(factory, node);

  std::cout << BT::writeTreeNodesModelXML(factory, true) << "\n";
  std::cout << "<!-- builtin_nodes:";
  for (const auto & id : factory.builtinNodes()) {
    std::cout << ' ' << id;
  }
  std::cout << " -->\n";

  rclcpp::shutdown();
  return 0;
}
