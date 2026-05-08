#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "oxebots_strategy/go_to_point_node.h"
#include "oxebots_strategy/kick_ball_node.h"
#include "oxebots_strategy/update_ball_position_node.h"
#include "oxebots_strategy/is_ball_close_condition.h"
#include "oxebots_strategy/goalkeeper_node.h"
#include "oxebots_interfaces/msg/role_assignment.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <thread>
#include <mutex>
#include <fstream>

class StrategyNode : public rclcpp::Node
{
public:
  StrategyNode() : Node("strategy_node")
  {
    this->declare_parameter<std::string>("bt_xml_path", "");
    this->declare_parameter<bool>("is_yellow_team", false);
    this->declare_parameter<int>("robot_id", 1);
    this->declare_parameter<double>("execution_rate", 60.0);

    is_yellow_ = this->get_parameter("is_yellow_team").as_bool();
    robot_id_ = this->get_parameter("robot_id").as_int();
    blackboard_ = BT::Blackboard::create();
    setup_blackboard();

    role_sub_ = this->create_subscription<oxebots_interfaces::msg::RoleAssignment>(
      "/role_assignment", 10, std::bind(&StrategyNode::role_callback, this, std::placeholders::_1));

    game_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
      "game_data", 10, std::bind(&StrategyNode::game_callback, this, std::placeholders::_1));
  }

  bool init()
  {
    try {
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("oxebots_strategy");

      // Agora o padrão é test_tree dentro da pasta behavior_trees
      std::string tree_path = this->get_parameter("bt_xml_path").as_string();
      if (tree_path.empty()) {
        tree_path = package_share_directory + "/behavior_trees/test_tree.xml";
      } else if (tree_path.find("/") == std::string::npos) {
        // Se o usuário passar apenas "simple_attack.xml", completamos o caminho
        tree_path = package_share_directory + "/behavior_trees/" + tree_path;
      }

      // Verificar se o arquivo existe
      std::ifstream file(tree_path);
      if (!file.good()) {
        RCLCPP_ERROR(this->get_logger(), "ARQUIVO NÃO ENCONTRADO: %s", tree_path.c_str());
        return false;
      }
      file.close();

      RCLCPP_INFO(this->get_logger(), "Carregando árvore: %s", tree_path.c_str());

      factory_.registerNodeType<oxebots_strategy::GoToPointNode>("GoToPoint", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::KickBallNode>("KickBall", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::UpdateBallPositionNode>("UpdateBallPosition", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::IsBallCloseCondition>("IsBallClose", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::GoalkeeperNode>("Goalkeeper", shared_from_this());

      // Registrar condição para verificar booleanos do blackboard
      factory_.registerSimpleCondition("IsValueTrue", [&](BT::TreeNode& node) {
          bool val;
          if (node.getInput("value", val)) {
              return val ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
          }
          return BT::NodeStatus::FAILURE;
      }, { BT::InputPort<bool>("value") });

      // Nova condição: IsMyRole(role="attacker" ou "defender")
      factory_.registerSimpleCondition("IsMyRole", [&](BT::TreeNode& node) {
          std::string role;
          uint32_t my_id, target_id;
          if (!node.getInput("role", role)) return BT::NodeStatus::FAILURE;
          if (!blackboard_->get("robot_id", my_id)) return BT::NodeStatus::FAILURE;

          if (role == "attacker") {
              if (blackboard_->get("attacker_id", target_id)) {
                  return (my_id == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
              }
          } else if (role == "defender") {
              if (blackboard_->get("defender_id", target_id)) {
                  return (my_id == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
              }
          }
          return BT::NodeStatus::FAILURE;
      }, { BT::InputPort<std::string>("role") });

      tree_ = factory_.createTreeFromFile(tree_path, blackboard_);
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "EXCEÇÃO NO INIT: %s", e.what());
      return false;
    }
  }

  void run()
  {
    double rate_hz;
    try {
        rate_hz = this->get_parameter("execution_rate").as_double();
    } catch (...) {
        rate_hz = 60.0;
    }

    if (rate_hz <= 0.1) rate_hz = 60.0;
    rclcpp::Rate rate(rate_hz);

    while (rclcpp::ok()) {
      if (has_data_) {
        try {
          tree_.tickOnce();
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Erro ao executar tick: %s", e.what());
        }
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Aguardando dados...");
      }

      try {
          rate.sleep();
      } catch (...) {
          // Fallback if clock jumps or rate fails
          std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0/rate_hz)));
      }
    }
  }

private:
  void setup_blackboard()
  {
    double my_goal_x = is_yellow_ ? 2200.0 : -2200.0;
    double opponent_goal_x = is_yellow_ ? -2200.0 : 2200.0;
    blackboard_->set("my_goal_x", my_goal_x);
    blackboard_->set("opponent_goal_x", opponent_goal_x);
    blackboard_->set("opponent_goal_y", 0.0);
    blackboard_->set("is_yellow", is_yellow_);
    blackboard_->set("attacker_id", static_cast<uint32_t>(1));
    blackboard_->set("defender_id", static_cast<uint32_t>(2));
    blackboard_->set("robot_id", static_cast<uint32_t>(robot_id_));
    blackboard_->set("is_goalkeeper", (robot_id_ == 0));
  }

  void role_callback(const oxebots_interfaces::msg::RoleAssignment::SharedPtr msg)
  {
    blackboard_->set("attacker_id", msg->attacker_id);
    blackboard_->set("defender_id", msg->defender_id);
  }

  void game_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
  {
    blackboard_->set("ball_x", msg->ball.x);
    blackboard_->set("ball_y", msg->ball.y);

    has_data_ = true;
  }

  bool is_yellow_;
  int robot_id_;
  std::atomic<bool> has_data_{false};
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  rclcpp::Subscription<oxebots_interfaces::msg::RoleAssignment>::SharedPtr role_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StrategyNode>();

  if (node->init()) {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    node->run();

    executor.cancel();
    if (executor_thread.joinable()) executor_thread.join();
  } else {
    RCLCPP_FATAL(node->get_logger(), "Falha crítica na inicialização do StrategyNode.");
  }

  rclcpp::shutdown();
  return 0;
}
