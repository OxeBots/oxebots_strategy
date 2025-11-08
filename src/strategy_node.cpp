#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "oxebots_strategy/go_to_point_node.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

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

  auto blackboard = BT::Blackboard::create();
  node->declare_parameter<bool>("is_yellow", false);
  bool is_yellow = node->get_parameter("is_yellow").as_bool();
  double opponent_goal_x = is_yellow ? -2200.0 : 2200.0;
  blackboard->set("opponent_goal_x", opponent_goal_x);
  blackboard->set("opponent_goal_y", 0.0);
  RCLCPP_INFO(node->get_logger(), "Time %s, gol do oponente em: (%.1f, 0.0)", is_yellow ? "amarelo" : "azul", opponent_goal_x);

  try
  {
    auto tree = factory.createTreeFromFile(tree_path, blackboard);

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
      tree.tickOnce();
      rclcpp::spin_some(node);
      rate.sleep();
    }
  }
  catch (const BT::RuntimeError& e)
  {
    RCLCPP_ERROR(node->get_logger(), "ERRO DE EXECUÇÃO DA ÁRVORE: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
