// Copyright 2024 Oxebots
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/blackboard.h"
#include "oxebots_strategy/go_to_point_node.h"
#include "oxebots_strategy/kick_ball_node.h"
#include "oxebots_strategy/update_ball_position_node.h"
#include "oxebots_strategy/is_ball_close_condition.h"
#include "oxebots_strategy/goalkeeper_node.h"
#include "oxebots_strategy/precision_kick_node.h"
#include "oxebots_strategy/defender_nodes.hpp"
#include "oxebots_interfaces/msg/role_assignment.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "ssl_league_msgs/msg/referee.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <fstream>
#include <filesystem>
#include <limits>

namespace fs = std::filesystem;

class StrategyNode : public rclcpp::Node
{
public:
  StrategyNode() : Node("strategy_node")
  {
    this->declare_parameter<std::string>("bt_xml_path", "");
    this->declare_parameter<bool>("is_yellow_team", false);
    this->declare_parameter<int>("robot_id", 1);
    this->declare_parameter<double>("execution_rate", 60.0);
    this->declare_parameter<std::string>("gc_topic", "/gc_multicast_bridge/referee_messages");
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

    gc_sub_ = this->create_subscription<ssl_league_msgs::msg::Referee>(
      this->get_parameter("gc_topic").as_string(), 10, std::bind(&StrategyNode::gc_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", 10);
  }

  bool init()
  {
    try {
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("oxebots_strategy");
      
      std::string tree_path = this->get_parameter("bt_xml_path").as_string();
      if (tree_path.empty()) {
        tree_path = package_share_directory + "/behavior_trees/master_strategy.xml";
      } else if (tree_path.find("/") == std::string::npos) {
        tree_path = package_share_directory + "/behavior_trees/" + tree_path;
      }

      RCLCPP_INFO(this->get_logger(), "Robô %d iniciando com árvore: %s", robot_id_, tree_path.c_str());

      // Configura o contexto de controle utilizado pelos nós da defender_tree.
      configureDefenderController(shared_from_this(), static_cast<uint32_t>(robot_id_));
      factory_.registerNodeType<oxebots_strategy::GoToPointNode>("GoToPoint", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::KickBallNode>("KickBall", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::UpdateBallPositionNode>("UpdateBallPosition", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::IsBallCloseCondition>("IsBallClose", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::GoalkeeperNode>("Goalkeeper", shared_from_this());
      factory_.registerNodeType<oxebots_strategy::PrecisionKickNode>("PrecisionKick", shared_from_this());

      // Nós da estratégia defensiva (defender_tree.xml)
      factory_.registerNodeType<IsBallInOpponentField>("IsBallInOpponentField");
      factory_.registerNodeType<IsOpponentControllingBall>("IsOpponentControllingBall");
      factory_.registerNodeType<ShadowBall>("ShadowBall");
      factory_.registerNodeType<MarkOpponent>("MarkOpponent");
      factory_.registerNodeType<GoToClamped>("GoToClamped");
      factory_.registerSimpleCondition("IsValueTrue", [&](BT::TreeNode& node) {
          bool val;
          if (node.getInput("value", val)) return val ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
          return BT::NodeStatus::FAILURE;
      }, { BT::InputPort<bool>("value") });

      factory_.registerSimpleCondition("IsThisNodeID", [&](BT::TreeNode& node) {
          unsigned int target_id;
          if (!node.getInput("id", target_id)) return BT::NodeStatus::FAILURE;
          return (static_cast<unsigned int>(robot_id_) == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }, { BT::InputPort<unsigned int>("id") });

      factory_.registerSimpleCondition("IsMyRole", [&](BT::TreeNode& node) {
          std::string role;
          uint32_t my_id, target_id;
          if (!node.getInput("role", role)) return BT::NodeStatus::FAILURE;
          if (!blackboard_->get("robot_id", my_id)) return BT::NodeStatus::FAILURE;
          if (role == "attacker") {
              if (blackboard_->get("attacker_id", target_id)) return (my_id == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
          } else if (role == "defender") {
              if (blackboard_->get("defender_id", target_id)) return (my_id == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
          }
          return BT::NodeStatus::FAILURE;
      }, { BT::InputPort<std::string>("role") });

      factory_.registerSimpleCondition("CheckGCCommand", [&](BT::TreeNode& node) {
          int expected;
          if (!node.getInput("expected", expected)) return BT::NodeStatus::FAILURE;
          int current;
          if (!blackboard_->get("gc_command", current)) return BT::NodeStatus::FAILURE;
          return (current == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }, { BT::InputPort<int>("expected") });

      factory_.registerSimpleCondition("IsRobotId", [&](BT::TreeNode& node) {
        uint32_t target_id, my_id;
        if (!node.getInput("robot_id", target_id)) return BT::NodeStatus::FAILURE;
        if (!blackboard_->get("robot_id", my_id)) return BT::NodeStatus::FAILURE;
        return (my_id == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }, { BT::InputPort<uint32_t>("robot_id") });

      factory_.registerSimpleAction("Halt", [&](BT::TreeNode& node) {
          std::string reason = "pelo GC";
          std::string input_reason;
          if (node.getInput("reason", input_reason) && input_reason == "idle") {
              reason = "por estar ocioso (sem papel)";
          }
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Robô %d parado %s", robot_id_, reason.c_str());
          
          auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
          oxebots_interfaces::msg::RobotCmdData cmd_data;
          cmd_data.id = robot_id_;
          cmd_data.x_velocity = 0.0;
          cmd_data.y_velocity = 0.0;
          cmd_data.angular_velocity = 0.0;
          cmd_msg->robots.push_back(cmd_data);
          cmd_pub_->publish(std::move(cmd_msg));
          return BT::NodeStatus::SUCCESS;
      }, { BT::InputPort<std::string>("reason") });

      // Registrar todas as árvores no diretório para permitir subárvores
      std::string bt_dir = package_share_directory + "/behavior_trees";
      if (fs::exists(bt_dir) && fs::is_directory(bt_dir)) {
          for (const auto & entry : fs::directory_iterator(bt_dir)) {
              if (entry.path().extension() == ".xml" && entry.path().string() != tree_path) {
                  try {
                      RCLCPP_INFO(this->get_logger(), "Registrando subárvore: %s", entry.path().filename().string().c_str());
                      factory_.registerBehaviorTreeFromFile(entry.path().string());
                  } catch (const std::exception& e) {
                      RCLCPP_WARN(this->get_logger(), "Falha ao registrar subárvore %s: %s", 
                                  entry.path().filename().string().c_str(), e.what());
                  }
              }
          }
      }

      tree_ = factory_.createTreeFromFile(tree_path, blackboard_);

      try {
        int port = 1667 + robot_id_;
        RCLCPP_INFO(this->get_logger(), "Iniciando Groot2 na porta %d...", port);
        publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, port);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Groot2 falhou na porta %d: %s", 1667 + robot_id_, e.what());
      }

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
    blackboard_->set("attacker_id", attacker_id_);
    blackboard_->set("defender_id", defender_id_);
    blackboard_->set("robot_id", static_cast<uint32_t>(robot_id_));
    blackboard_->set("is_goalkeeper", (robot_id_ == 0));
    blackboard_->set("gc_command", -1);
    blackboard_->set("gc_stage", -1);
    blackboard_->set("is_free_kick", false);

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

    // Atualizar posição do robô (HEAD)
    for (const auto& robot : msg->robots.allies) {
      if (robot.id == self_id) {
        blackboard_->set("robot_x", static_cast<double>(robot.x));
        blackboard_->set("robot_y", static_cast<double>(robot.y));
        blackboard_->set("robot_yaw", static_cast<double>(robot.orientation));
        break;
      }
    }

    // Lógica da branch defender
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

  void gc_callback(const ssl_league_msgs::msg::Referee::SharedPtr msg)
  {
    int current_cmd;
    if (!blackboard_->get<int>("gc_command", current_cmd) || current_cmd != static_cast<int>(msg->command)) {
        RCLCPP_INFO(this->get_logger(), "Robô %d -> Novo comando do Juiz: %d", robot_id_, msg->command);
    }

    blackboard_->set("gc_command", static_cast<int>(msg->command));
    blackboard_->set("gc_stage", static_cast<int>(msg->stage));

    // Lógica para Falta trazida da branch defender (adaptada para o comando do GC padrão da liga)
    // 8: DIRECT_FREE_YELLOW, 9: DIRECT_FREE_BLUE
    bool our_foul = (is_yellow_ && msg->command == 8) || (!is_yellow_ && msg->command == 9);
    blackboard_->set("is_free_kick", our_foul);

    // O bridge da A-TEAM envia designated_position como um array opcional
    if (!msg->designated_position.empty()) {
        blackboard_->set("designated_x", static_cast<double>(msg->designated_position[0].x * 1000.0));
        blackboard_->set("designated_y", static_cast<double>(msg->designated_position[0].y * 1000.0));
    } else {
        blackboard_->set("designated_x", 0.0);
        blackboard_->set("designated_y", 0.0);
    }
  }

  bool is_yellow_;
  int robot_id_;
  uint32_t attacker_id_{2};
  uint32_t defender_id_{1};

  double possession_distance_mm_{220.0};

  bool has_last_ball_{false};
  Pose2D last_ball_{0.0, 0.0, 0.0};
  rclcpp::Time last_ball_time_;

  std::atomic<bool> has_data_{false};
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> publisher_;
  BT::Blackboard::Ptr blackboard_;
  rclcpp::Subscription<oxebots_interfaces::msg::RoleAssignment>::SharedPtr role_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_sub_;
  rclcpp::Subscription<ssl_league_msgs::msg::Referee>::SharedPtr gc_sub_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_pub_;
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
