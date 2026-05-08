#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "oxebots_strategy/go_to_point_node.h"
#include "oxebots_strategy/kick_ball_node.h"
#include "oxebots_strategy/update_ball_position_node.h"
#include "oxebots_strategy/is_ball_close_condition.h"
#include "oxebots_strategy/goalkeeper_node.h"
#include "oxebots_strategy/defender_nodes.hpp"
#include "oxebots_interfaces/msg/role_assignment.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/referee.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <fstream>
#include <limits>

class StrategyNode : public rclcpp::Node
{
public:
  StrategyNode() : Node("strategy_node")
  {
    this->declare_parameter<std::string>("bt_xml_path", "");
    this->declare_parameter<bool>("is_yellow_team", false);
    this->declare_parameter<int>("robot_id", 1);
    this->declare_parameter<double>("execution_rate", 60.0);
    this->declare_parameter<double>("possession_distance", 220.0);

    is_yellow_ = this->get_parameter("is_yellow_team").as_bool();
    robot_id_ = this->get_parameter("robot_id").as_int();
    possession_distance_mm_ = this->get_parameter("possession_distance").as_double();

    blackboard_ = BT::Blackboard::create();
    setup_blackboard();

    role_sub_ = this->create_subscription<oxebots_interfaces::msg::RoleAssignment>(
      "/role_assignment", 10, std::bind(&StrategyNode::role_callback, this, std::placeholders::_1));
    
    game_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
      "game_data", 10, std::bind(&StrategyNode::game_callback, this, std::placeholders::_1));
    referee_sub_ = this->create_subscription<oxebots_interfaces::msg::Referee>(
  "/referee", 10, std::bind(&StrategyNode::referee_callback, this, std::placeholders::_1));
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

      // Configura o contexto de controle utilizado pelos nós da defender_tree.
      configureDefenderController(shared_from_this(), static_cast<uint32_t>(robot_id_));

      factory_.registerNodeType<oxebots_strategy::GoToPointNode>("GoToPoint", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::KickBallNode>("KickBall", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::UpdateBallPositionNode>("UpdateBallPosition", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::IsBallCloseCondition>("IsBallClose", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::GoalkeeperNode>("Goalkeeper", shared_from_this());

      // Nós da estratégia defensiva (defender_tree.xml)
      factory_.registerNodeType<IsBallInOpponentField>("IsBallInOpponentField");
      factory_.registerNodeType<IsOpponentControllingBall>("IsOpponentControllingBall");
      factory_.registerNodeType<ShadowBall>("ShadowBall");
      factory_.registerNodeType<MarkOpponent>("MarkOpponent");
      factory_.registerNodeType<GoToClamped>("GoToClamped");

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

          // Condição para seleção manual do robô no XML da BT.
          factory_.registerSimpleCondition("IsRobotId", [&](BT::TreeNode& node) {
            uint32_t target_id, my_id;
            if (!node.getInput("robot_id", target_id)) return BT::NodeStatus::FAILURE;
            if (!blackboard_->get("robot_id", my_id)) return BT::NodeStatus::FAILURE;
            return (my_id == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
          }, { BT::InputPort<uint32_t>("robot_id") });

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
  void referee_callback(const oxebots_interfaces::msg::Referee::SharedPtr msg)
  {
    // Lógica para Falta (Ex: 8: Yellow, 9: Blue)
    bool our_foul = (is_yellow_ && msg->command == 8) || (!is_yellow_ && msg->command == 9);
    
    // Seta no blackboard para a árvore ler
    blackboard_->set("is_free_kick", our_foul);
    
  }
  rclcpp::Subscription<oxebots_interfaces::msg::Referee>::SharedPtr referee_sub_;

  void setup_blackboard()
  {
    double my_goal_x = is_yellow_ ? 2200.0 : -2200.0;
    double opponent_goal_x = is_yellow_ ? -2200.0 : 2200.0;
    blackboard_->set("my_goal_x", my_goal_x);
    blackboard_->set("opponent_goal_x", opponent_goal_x);
    blackboard_->set("opponent_goal_y", 0.0);
    blackboard_->set("is_yellow", is_yellow_);
    blackboard_->set("attacker_id", attacker_id_);
    blackboard_->set("defender_id", defender_id_);
    blackboard_->set("robot_id", static_cast<uint32_t>(robot_id_));
    blackboard_->set("is_goalkeeper", (robot_id_ == 0));

    blackboard_->set("ball_x", 0.0);
    blackboard_->set("ball_y", 0.0);
    blackboard_->set("ball_vx", 0.0);
    blackboard_->set("ball_vy", 0.0);
    blackboard_->set("ball", Pose2D{0.0, 0.0, 0.0});
    blackboard_->set("ball_vel", Vector2D{0.0, 0.0});
    blackboard_->set("opponent_controlling", false);
    blackboard_->set("attacker_ball_dist", std::numeric_limits<double>::infinity());
    blackboard_->set("defender_ball_dist", std::numeric_limits<double>::infinity());
  }

  void role_callback(const oxebots_interfaces::msg::RoleAssignment::SharedPtr msg)
  {
    blackboard_->set("attacker_id", msg->attacker_id);
    blackboard_->set("defender_id", msg->defender_id);
    uint32_t my_id = blackboard_->get<uint32_t>("robot_id");

    // Define se este robô específico é o atacante (o batedor da falta)
    blackboard_->set("is_attacker", (my_id == msg->attacker_id));
  }

  void game_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
  {
    const double ball_x = static_cast<double>(msg->ball.x);
    const double ball_y = static_cast<double>(msg->ball.y);
    const uint32_t self_id = static_cast<uint32_t>(robot_id_);

    double ball_vx = 0.0;
    double ball_vy = 0.0;

    const auto now_time = this->now();
    if (has_last_ball_) {
      const double dt = (now_time - last_ball_time_).seconds();
      if (dt > 1e-4) {
        ball_vx = (ball_x - last_ball_.x) / dt;
        ball_vy = (ball_y - last_ball_.y) / dt;
      }
    }

    last_ball_ = Pose2D{ball_x, ball_y, 0.0};
    last_ball_time_ = now_time;
    has_last_ball_ = true;

    double min_ally_dist = std::numeric_limits<double>::infinity();
    double min_enemy_dist = std::numeric_limits<double>::infinity();
    double attacker_ball_dist = std::numeric_limits<double>::infinity();
    double defender_ball_dist = std::numeric_limits<double>::infinity();

    for (const auto& ally : msg->robots.allies) {
      const double dist = std::hypot(ally.x - ball_x, ally.y - ball_y);
      min_ally_dist = std::min(min_ally_dist, dist);

      if (ally.id == attacker_id_) {
        attacker_ball_dist = dist;
      }
      // Para a defender_tree, defensor = este nó (robô atual).
      if (ally.id == self_id) {
        defender_ball_dist = dist;
      }
    }

    for (const auto& enemy : msg->robots.enemies) {
      const double dist = std::hypot(enemy.x - ball_x, enemy.y - ball_y);
      min_enemy_dist = std::min(min_enemy_dist, dist);
    }

    const bool opponent_controlling =
      std::isfinite(min_enemy_dist) &&
      (min_enemy_dist <= possession_distance_mm_) &&
      (!std::isfinite(min_ally_dist) || (min_enemy_dist + 30.0 < min_ally_dist));

    blackboard_->set("ball_x", ball_x);
    blackboard_->set("ball_y", ball_y);
    blackboard_->set("ball_vx", ball_vx);
    blackboard_->set("ball_vy", ball_vy);
    blackboard_->set("ball", Pose2D{ball_x, ball_y, 0.0});
    blackboard_->set("ball_vel", Vector2D{ball_vx, ball_vy});
    blackboard_->set("opponent_controlling", opponent_controlling);
    blackboard_->set("attacker_ball_dist", attacker_ball_dist);
    blackboard_->set("defender_ball_dist", defender_ball_dist);

    has_data_.store(true);
  }

  bool is_yellow_;
  int robot_id_;
  uint32_t attacker_id_{1};
  uint32_t defender_id_{2};

  double possession_distance_mm_{220.0};

  bool has_last_ball_{false};
  Pose2D last_ball_{0.0, 0.0, 0.0};
  rclcpp::Time last_ball_time_;

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
