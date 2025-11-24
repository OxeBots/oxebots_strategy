#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "oxebots_strategy/go_to_point_node.h"
#include "oxebots_strategy/kick_ball_node.h"
#include "oxebots_strategy/update_ball_position_node.h"
#include "oxebots_strategy/is_ball_close_condition.h"
#include "oxebots_strategy/goalkeeper_node.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <thread>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("strategy_node");

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("oxebots_strategy");
  std::string default_tree_path = package_share_directory + "/test_tree.xml";
  
  node->declare_parameter<std::string>("bt_xml_path", default_tree_path);
  std::string tree_path = node->get_parameter("bt_xml_path").as_string();

  RCLCPP_INFO(node->get_logger(), "Carregando árvore de comportamento de: %s", tree_path.c_str());

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<oxebots_strategy::GoToPointNode>("GoToPoint", node);
  factory.registerNodeType<oxebots_strategy::KickBallNode>("KickBall", node);
  factory.registerNodeType<oxebots_strategy::UpdateBallPositionNode>("UpdateBallPosition", node);
  factory.registerNodeType<oxebots_strategy::IsBallCloseCondition>("IsBallClose", node);
  factory.registerNodeType<oxebots_strategy::GoalkeeperNode>("Goalkeeper", node);

  auto blackboard = BT::Blackboard::create();
  node->declare_parameter<bool>("is_yellow", false);
  bool is_yellow = node->get_parameter("is_yellow").as_bool();
  
  double my_goal_x = is_yellow ? 2200.0 : -2200.0;
  double opponent_goal_x = is_yellow ? -2200.0 : 2200.0;
  
  blackboard->set("my_goal_x", my_goal_x);
  blackboard->set("opponent_goal_x", opponent_goal_x);
  blackboard->set("opponent_goal_y", 0.0);
  blackboard->set("is_yellow", is_yellow);
  blackboard->set("robot_id", 0);

  RCLCPP_INFO(node->get_logger(), "Time %s, meu gol em X: %.1f, gol do oponente em: (%.1f, 0.0)", is_yellow ? "amarelo" : "azul", my_goal_x, opponent_goal_x);

  // Executor ROS em uma thread separada
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  try
  {
    auto tree = factory.createTreeFromFile(tree_path, blackboard);

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
      tree.tickOnce();
      rate.sleep();
    }
  }
  catch (const BT::RuntimeError& e)
  {
    RCLCPP_ERROR(node->get_logger(), "ERRO DE EXECUÇÃO DA ÁRVORE: %s", e.what());
  }

  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}
