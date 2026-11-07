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
#include "oxebots_strategy/defender_nodes.hpp"
#include "oxebots_strategy/node_registry.hpp"
#include "oxebots_interfaces/msg/role_assignment.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/game_state.hpp"
#include "oxebots_interfaces/msg/robot_motion_override.hpp"
#include "oxebots_interfaces/msg/ssl_geometry_data.hpp"
#include "ssl_league_msgs/msg/referee.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <fstream>
#include <filesystem>
#include <limits>
#include <vector>

namespace fs = std::filesystem;

class StrategyNode : public rclcpp::Node
{
public:
  explicit StrategyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("strategy_node", options)
  {
    this->declare_parameter<std::string>("bt_xml_path", "");
    this->declare_parameter<bool>("is_yellow_team", false);
    this->declare_parameter<bool>("invert_sides", false);
    this->declare_parameter<int>("robot_id", 1);
    this->declare_parameter<double>("execution_rate", 60.0);
    this->declare_parameter<std::string>("game_state_topic", "/game_state");
    this->declare_parameter<std::string>("gc_topic", "/gc_multicast_bridge/referee_messages");
    this->declare_parameter<double>("possession_distance", 220.0);
    this->declare_parameter<bool>("require_gc", true);
    this->declare_parameter<double>("max_gc_ms", 500.0);
    this->declare_parameter<double>("max_pose_ms", 150.0);
    this->declare_parameter<double>("max_ball_ms", 300.0);

    is_yellow_ = this->get_parameter("is_yellow_team").as_bool();
    invert_sides_ = this->get_parameter("invert_sides").as_bool();
    robot_id_ = this->get_parameter("robot_id").as_int();
    possession_distance_mm_ = this->get_parameter("possession_distance").as_double();
    require_gc_ = this->get_parameter("require_gc").as_bool();
    max_gc_ms_ = this->get_parameter("max_gc_ms").as_double();
    max_pose_ms_ = this->get_parameter("max_pose_ms").as_double();
    max_ball_ms_ = this->get_parameter("max_ball_ms").as_double();

    blackboard_ = BT::Blackboard::create();
    setup_blackboard();

    role_sub_ = this->create_subscription<oxebots_interfaces::msg::RoleAssignment>(
      "/role_assignment", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&StrategyNode::role_callback, this, std::placeholders::_1));
    
    game_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
      "game_data", 10, std::bind(&StrategyNode::game_callback, this, std::placeholders::_1));

    game_state_sub_ = this->create_subscription<oxebots_interfaces::msg::GameState>(
      this->get_parameter("game_state_topic").as_string(),
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&StrategyNode::game_state_callback, this, std::placeholders::_1));

    gc_sub_ = this->create_subscription<ssl_league_msgs::msg::Referee>(
      this->get_parameter("gc_topic").as_string(), 10, std::bind(&StrategyNode::gc_callback, this, std::placeholders::_1));
    
    // Assina a geometria oficial do campo publicada pelo game_observer_node
    geometry_sub_ = this->create_subscription<oxebots_interfaces::msg::SSLGeometryData>(
      "/field_geometry", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&StrategyNode::geometry_callback, this, std::placeholders::_1));

    override_pub_ = this->create_publisher<oxebots_interfaces::msg::RobotMotionOverride>("/robot_motion_override", 10);
    active_branch_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/strategy/active_branch", rclcpp::QoS(10).transient_local().reliable());
  }

  BT::Blackboard::Ptr getBlackboard() const { return blackboard_; }
  const std::string & getActiveBranch() const { return active_branch_; }
  void setTree(BT::Tree && tree) {
    tree_ = std::move(tree);
    attachBranchObserver();
  }
  void setLastGameState(const oxebots_interfaces::msg::GameState::SharedPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_game_state_ = msg;
  }
  void setLastGeometry(const oxebots_interfaces::msg::SSLGeometryData::SharedPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_geometry_ = msg;
    has_geometry_.store(true);
  }
  void setLastGameData(const oxebots_interfaces::msg::GameData::SharedPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_game_data_ = msg;
    last_game_data_time_ = this->now();
    has_data_.store(true);
  }
  void setLastRoleAssignment(const oxebots_interfaces::msg::RoleAssignment::SharedPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_role_assignment_ = msg;
  }

  bool init()
  {
    try {
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("oxebots_strategy");
      
      std::string tree_path = this->get_parameter("bt_xml_path").as_string();
      if (tree_path.empty()) {
        tree_path = package_share_directory + "/behavior_trees/master_strategy_carry.xml";
      } else if (tree_path.find("/") == std::string::npos) {
        tree_path = package_share_directory + "/behavior_trees/" + tree_path;
      }

      RCLCPP_INFO(this->get_logger(), "Robô %d iniciando com árvore: %s", robot_id_, tree_path.c_str());

      oxebots_strategy::registerStrategyNodes(
        factory_, shared_from_this(), blackboard_, override_pub_, robot_id_);

      // Registrar todas as árvores no diretório para permitir subárvores
      std::string bt_dir = package_share_directory + "/behavior_trees";
      if (fs::exists(bt_dir) && fs::is_directory(bt_dir)) {
          for (const auto & entry : fs::directory_iterator(bt_dir)) {
              if (entry.path().extension() == ".xml" &&
                  entry.path().string() != tree_path &&
                  entry.path().filename() != "tree_nodes_model.xml") {
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
      attachBranchObserver();

      try {
        int port = (is_yellow_ ? 1677 : 1667) + robot_id_;
        RCLCPP_INFO(this->get_logger(), "Iniciando Groot2 na porta %d...", port);
        publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, port);
      } catch (const std::exception& e) {
        int port = (is_yellow_ ? 1677 : 1667) + robot_id_;
        RCLCPP_WARN(this->get_logger(), "Groot2 falhou na porta %d: %s", port, e.what());
      }

      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "EXCEÇÃO NO INIT: %s", e.what());
      return false;
    }
  }

  BT::NodeStatus tickOnce()
  {
    tick_selected_branch_.clear();
    const BT::NodeStatus status = tree_.tickOnce();

    std::string selected_branch;
    for (BT::TreeNode * branch : root_branches_) {
      if (branch != nullptr && branch->status() == BT::NodeStatus::RUNNING) {
        selected_branch = branch->name();
        break;
      }
    }
    if (selected_branch.empty()) {
      selected_branch = tick_selected_branch_;
    }

    if (!selected_branch.empty() && selected_branch != active_branch_) {
      const std::string prev_branch = active_branch_.empty() ? "NONE" : active_branch_;
      active_branch_ = selected_branch;

      const std::string phase_str = currentPhaseString();
      std::string role_str = "idle";
      (void)blackboard_->get("role", role_str);
      if (role_str.empty()) {
        role_str = "idle";
      }

      const std::string reason = resolveTransitionReason(
        prev_branch, active_branch_, phase_str, role_str);

      RCLCPP_INFO(
        this->get_logger(),
        "Transição de ramo [robô %d]: %s -> %s (fase=%s, papel=%s, motivo=%s)",
        robot_id_,
        prev_branch.c_str(),
        active_branch_.c_str(),
        phase_str.c_str(),
        role_str.c_str(),
        reason.c_str());

      if (active_branch_pub_) {
        std_msgs::msg::String msg;
        msg.data =
          "robot_id=" + std::to_string(robot_id_) +
          " branch=" + active_branch_ +
          " prev_branch=" + prev_branch +
          " phase=" + phase_str +
          " role=" + role_str +
          " reason=" + reason;
        active_branch_pub_->publish(msg);
      }
    }

    return status;
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
      if (has_data_ && has_geometry_) {
        try {
          refresh();
          tickOnce();
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Erro ao executar tick: %s", e.what());
        }
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Aguardando dados (game_data=%s, field_geometry=%s)...",
                             has_data_.load() ? "ok" : "pendente",
                             has_geometry_.load() ? "ok" : "pendente");
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
    // D03: is_yellow volta a significar a cor real do time (invert_sides fica exclusivo da visão/simulador)
    blackboard_->set("is_yellow", is_yellow_);
    blackboard_->set("goalkeeper_id", goalkeeper_id_);
    blackboard_->set("attacker_id", attacker_id_);
    blackboard_->set("defender_id", defender_id_);
    blackboard_->set("kicker_id", kicker_id_);
    blackboard_->set("robot_id", static_cast<uint32_t>(robot_id_));
    blackboard_->set("is_goalkeeper", false);
    blackboard_->set("is_attacker", false);
    blackboard_->set("is_defender", false);
    blackboard_->set("is_kicker", false);
    blackboard_->set("role", std::string("idle"));
    blackboard_->set("role_epoch", 0u);
    blackboard_->set("active_ids", std::vector<uint32_t>{});

    // Chaves de geometria e lado do campo (Capítulo 20 §2)
    blackboard_->set("own_goal_x", 0.0);
    blackboard_->set("opp_goal_x", 0.0);
    blackboard_->set("opp_goal_y", 0.0);
    blackboard_->set("side_sign", 1);
    blackboard_->set("my_goal_x", 0.0);
    blackboard_->set("opponent_goal_x", 0.0);
    blackboard_->set("opponent_goal_y", 0.0);
    blackboard_->set("field_length", 0.0);
    blackboard_->set("field_width", 0.0);
    blackboard_->set("penalty_area_depth", 0.0);
    blackboard_->set("penalty_area_width", 0.0);
    blackboard_->set("goal_width", 0.0);
    blackboard_->set("center_circle_radius", 0.0);

    // Configurações de limites e obrigatoriedade de GC
    blackboard_->set("require_gc", require_gc_);
    blackboard_->set("use_gc", require_gc_);
    blackboard_->set("max_gc_ms", max_gc_ms_);
    blackboard_->set("max_pose_ms", max_pose_ms_);
    blackboard_->set("max_ball_ms", max_ball_ms_);

    // Chaves de GameState e GC
    const int initial_phase = require_gc_ ? 0 : 9;  // 9 = PHASE_RUN no modo sem GC
    blackboard_->set("phase", initial_phase);
    blackboard_->set("restart_ctx", 0);
    blackboard_->set("we_favored", false);
    blackboard_->set("speed_limit", 1.5);
    blackboard_->set("can_place_ball", false);
    blackboard_->set("has_designated", false);
    blackboard_->set("designated", Pose2D{0.0, 0.0, 0.0});
    blackboard_->set("designated_x", 0.0);
    blackboard_->set("designated_y", 0.0);
    blackboard_->set("gc_command", -1);
    blackboard_->set("gc_stage", -1);
    blackboard_->set("is_free_kick", false);

    // Poses e velocidades
    blackboard_->set("robot_x", 0.0);
    blackboard_->set("robot_y", 0.0);
    blackboard_->set("robot_yaw", 0.0);
    blackboard_->set("robot_pose", Pose2D{0.0, 0.0, 0.0});

    blackboard_->set("ball_x", 0.0);
    blackboard_->set("ball_y", 0.0);
    blackboard_->set("ball_vx", 0.0);
    blackboard_->set("ball_vy", 0.0);
    blackboard_->set("ball", Pose2D{0.0, 0.0, 0.0});
    blackboard_->set("ball_vel", Vector2D{0.0, 0.0});
    blackboard_->set("opponent_controlling", false);
    blackboard_->set("attacker_ball_dist", std::numeric_limits<double>::infinity());
    blackboard_->set("defender_ball_dist", std::numeric_limits<double>::infinity());

    // Idades dos dados
    blackboard_->set("gc_age_ms", require_gc_ ? std::numeric_limits<double>::infinity() : 0.0);
    blackboard_->set("pose_age_ms", std::numeric_limits<double>::infinity());
    blackboard_->set("ball_age_ms", std::numeric_limits<double>::infinity());
  }

  void role_callback(const oxebots_interfaces::msg::RoleAssignment::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_role_assignment_ = msg;
  }

  void game_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_game_data_ = msg;
    last_game_data_time_ = this->now();
    has_data_.store(true);
  }

  void game_state_callback(const oxebots_interfaces::msg::GameState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_game_state_ = msg;
  }

  void gc_callback(const ssl_league_msgs::msg::Referee::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_referee_ = msg;
  }

  void geometry_callback(const oxebots_interfaces::msg::SSLGeometryData::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_geometry_ = msg;
    has_geometry_.store(true);
  }

public:
  void refresh()
  {
    oxebots_interfaces::msg::GameState::SharedPtr gs;
    oxebots_interfaces::msg::GameData::SharedPtr gd;
    oxebots_interfaces::msg::SSLGeometryData::SharedPtr geom;
    oxebots_interfaces::msg::RoleAssignment::SharedPtr roles;
    ssl_league_msgs::msg::Referee::SharedPtr ref;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      gs = last_game_state_;
      gd = last_game_data_;
      geom = last_geometry_;
      roles = last_role_assignment_;
      ref = last_referee_;
    }

    const auto now_time = this->now();

    // 1. Cor real: D03 determina que is_yellow volta a significar apenas a cor real do time
    blackboard_->set("is_yellow", is_yellow_);

    // 2. Geometria e Lado do Campo (Capítulo 20 §O lado do campo)
    if (geom) {
      const double field_length = static_cast<double>(geom->field.field_length);
      const double field_width = static_cast<double>(geom->field.field_width);
      const double half_len = field_length / 2.0;

      // our_goal_positive_x é calculado pelo referee_state_node a partir de blue_team_on_positive_half.
      // Se GameState ainda não chegou, fallback padrão: amarelo em +X, azul em -X.
      const bool our_positive = gs ? gs->our_goal_positive_x : is_yellow_;

      // own_goal_x = +L/2 se nosso gol está em x positivo, -L/2 caso contrário
      const double own_goal_x = our_positive ? half_len : -half_len;
      const double opp_goal_x = -own_goal_x;

      // side_sign = +1 se defendemos o gol de x negativo (nossa metade é x < 0), -1 caso contrário
      const int side_sign = our_positive ? -1 : 1;

      blackboard_->set("field_length", field_length);
      blackboard_->set("field_width", field_width);
      blackboard_->set("penalty_area_depth", static_cast<double>(geom->field.penalty_area_depth));
      blackboard_->set("penalty_area_width", static_cast<double>(geom->field.penalty_area_width));
      blackboard_->set("goal_width", static_cast<double>(geom->field.goal_width));
      blackboard_->set("center_circle_radius", static_cast<double>(geom->field.center_circle_radius));

      blackboard_->set("own_goal_x", own_goal_x);
      blackboard_->set("opp_goal_x", opp_goal_x);
      blackboard_->set("opp_goal_y", 0.0);
      blackboard_->set("side_sign", side_sign);

      // Compatibilidade legada com árvores e folhas que leem my_goal_x / opponent_goal_*
      blackboard_->set("my_goal_x", own_goal_x);
      blackboard_->set("opponent_goal_x", opp_goal_x);
      blackboard_->set("opponent_goal_y", 0.0);
    }

    // 3. GameState e comandos de arbitragem
    if (gs) {
      blackboard_->set("phase", static_cast<int>(gs->phase));
      blackboard_->set("restart_ctx", static_cast<int>(gs->restart_ctx));
      blackboard_->set("we_favored", gs->we_favored);
      blackboard_->set("speed_limit", static_cast<double>(gs->speed_limit_mps));
      blackboard_->set("can_place_ball", gs->can_place_ball);
      blackboard_->set("has_designated", gs->has_designated);
      blackboard_->set("designated", Pose2D{static_cast<double>(gs->designated_position.x),
                                            static_cast<double>(gs->designated_position.y),
                                            0.0});

      // Compatibilidade legada com árvores existentes que leem gc_command / gc_stage
      blackboard_->set("gc_command", static_cast<int>(gs->command));
      blackboard_->set("gc_stage", static_cast<int>(gs->stage));
      blackboard_->set("is_free_kick", (gs->phase == oxebots_interfaces::msg::GameState::PHASE_FREE_KICK && gs->we_favored));
      if (gs->has_designated) {
        blackboard_->set("designated_x", static_cast<double>(gs->designated_position.x));
        blackboard_->set("designated_y", static_cast<double>(gs->designated_position.y));
      }

      double gc_age_ms = (now_time - rclcpp::Time(gs->header.stamp)).nanoseconds() / 1e6;
      if (gc_age_ms < 0.0) gc_age_ms = 0.0;
      // O referee_state_node recarimba o GameState a cada publicação, inclusive as de 100 ms com
      // stale=true; o carimbo sozinho não revela a perda do GC (Cap. 15 e 17).
      if (gs->stale) gc_age_ms = std::numeric_limits<double>::infinity();
      blackboard_->set("gc_age_ms", gc_age_ms);
    } else if (ref) {
      blackboard_->set("gc_command", static_cast<int>(ref->command));
      blackboard_->set("gc_stage", static_cast<int>(ref->stage));
      bool our_foul = (is_yellow_ && ref->command == 8) || (!is_yellow_ && ref->command == 9);
      blackboard_->set("is_free_kick", our_foul);
      if (!ref->designated_position.empty()) {
        double dx = static_cast<double>(ref->designated_position[0].x * 1000.0);
        double dy = static_cast<double>(ref->designated_position[0].y * 1000.0);
        blackboard_->set("designated_x", dx);
        blackboard_->set("designated_y", dy);
        blackboard_->set("has_designated", true);
        blackboard_->set("designated", Pose2D{dx, dy, 0.0});
      }
      blackboard_->set("gc_age_ms", 0.0);

      // Mapeamento de phase a partir do referee bruto para garantir compatibilidade
      int mapped_phase = 0;
      switch (ref->command) {
        case 0: mapped_phase = 0; break;  // HALT
        case 1: mapped_phase = 2; break;  // STOP
        case 2: mapped_phase = 9; break;  // NORMAL_START -> RUN
        case 3: mapped_phase = 9; break;  // FORCE_START -> RUN
        case 4: case 5: mapped_phase = 4; break;  // PREPARE_KICKOFF
        case 6: case 7: mapped_phase = 6; break;  // PREPARE_PENALTY
        case 8: case 9: case 10: case 11: mapped_phase = 8; break;  // FREE_KICK
        case 12: case 13: mapped_phase = 1; break;  // TIMEOUT
        case 16: case 17: mapped_phase = 3; break;  // BALL_PLACEMENT
        default: mapped_phase = 0; break;
      }
      blackboard_->set("phase", mapped_phase);
      bool we_favored = (is_yellow_ && (ref->command == 4 || ref->command == 6 || ref->command == 8 || ref->command == 10 || ref->command == 16)) ||
                        (!is_yellow_ && (ref->command == 5 || ref->command == 7 || ref->command == 9 || ref->command == 11 || ref->command == 17));
      blackboard_->set("we_favored", we_favored);
    } else if (!require_gc_) {
      blackboard_->set("phase", 9);  // PHASE_RUN
      blackboard_->set("gc_age_ms", 0.0);
    }

    // 4. Papéis (RoleAssignment) — atualizado antes de GameData para que attacker_ball_dist use o attacker_id_ atual
    if (roles) {
      goalkeeper_id_ = roles->goalkeeper_id;
      attacker_id_ = roles->attacker_id;
      defender_id_ = roles->defender_id;
      kicker_id_ = roles->kicker_id;

      blackboard_->set("goalkeeper_id", roles->goalkeeper_id);
      blackboard_->set("attacker_id", roles->attacker_id);
      blackboard_->set("defender_id", roles->defender_id);
      blackboard_->set("kicker_id", roles->kicker_id);
      blackboard_->set("role_epoch", roles->epoch);
      blackboard_->set("active_ids", roles->active_ids);

      const uint32_t my_id = static_cast<uint32_t>(robot_id_);
      std::string my_role = "idle";
      if (my_id == roles->goalkeeper_id) {
        my_role = "goalkeeper";
      } else if (my_id == roles->attacker_id) {
        my_role = "attacker";
      } else if (my_id == roles->defender_id) {
        my_role = "defender";
      }
      blackboard_->set("role", my_role);
      blackboard_->set("is_goalkeeper", (my_id == roles->goalkeeper_id));
      blackboard_->set("is_attacker", (my_id == roles->attacker_id));
      blackboard_->set("is_defender", (my_id == roles->defender_id));
      blackboard_->set("is_kicker", (my_id == roles->kicker_id));
    }

    // 5. GameData (robôs, bola, velocidades e idades)
    if (gd) {
      const double ball_x = static_cast<double>(gd->ball.x);
      const double ball_y = static_cast<double>(gd->ball.y);
      const uint32_t self_id = static_cast<uint32_t>(robot_id_);

      // Atualizar posição do robô
      bool found_self = false;
      for (const auto& robot : gd->robots.allies) {
        if (robot.id == self_id) {
          found_self = true;
          const double rx = static_cast<double>(robot.x);
          const double ry = static_cast<double>(robot.y);
          const double ryaw = static_cast<double>(robot.orientation);

          blackboard_->set("robot_x", rx);
          blackboard_->set("robot_y", ry);
          blackboard_->set("robot_yaw", ryaw);
          blackboard_->set("robot_pose", Pose2D{rx, ry, ryaw});

          double pose_age_ms = (now_time - last_game_data_time_).nanoseconds() / 1e6;
          if (pose_age_ms < 0.0) pose_age_ms = 0.0;
          blackboard_->set("pose_age_ms", pose_age_ms);
          break;
        }
      }
      if (!found_self) {
        blackboard_->set("pose_age_ms", std::numeric_limits<double>::infinity());
      }

      // Estimação de velocidade da bola (mm/s). O game_data chega várias vezes por quadro de visão
      // com a MESMA posição (~60% das mensagens), então derivar a cada refresh dava velocidades que
      // alternavam entre 0 e o dobro do valor real. Aqui a velocidade só é atualizada quando a posição
      // MUDOU, usando o tempo desde a última posição distinta, e passa por um filtro passa-baixa.
      // Posição parada por mais de kBallStillS -> velocidade 0; salto absurdo (bola teletransportada
      // ou leitura ruim) -> reancora sem gerar velocidade.
      constexpr double kBallMinDtS = 0.008;
      constexpr double kBallStillS = 0.10;
      constexpr double kBallMaxSpeedMmS = 8000.0;
      constexpr double kBallVelAlpha = 0.5;
      if (!has_last_ball_) {
        last_ball_ = Pose2D{ball_x, ball_y, 0.0};
        last_ball_time_ = now_time;
        has_last_ball_ = true;
        ball_vx_filt_ = 0.0;
        ball_vy_filt_ = 0.0;
      } else {
        const double dt = (now_time - last_ball_time_).seconds();
        if (ball_x != last_ball_.x || ball_y != last_ball_.y) {
          if (dt > kBallMinDtS) {
            const double raw_vx = (ball_x - last_ball_.x) / dt;
            const double raw_vy = (ball_y - last_ball_.y) / dt;
            if (std::hypot(raw_vx, raw_vy) > kBallMaxSpeedMmS) {
              ball_vx_filt_ = 0.0;
              ball_vy_filt_ = 0.0;
            } else {
              ball_vx_filt_ = kBallVelAlpha * raw_vx + (1.0 - kBallVelAlpha) * ball_vx_filt_;
              ball_vy_filt_ = kBallVelAlpha * raw_vy + (1.0 - kBallVelAlpha) * ball_vy_filt_;
            }
            last_ball_ = Pose2D{ball_x, ball_y, 0.0};
            last_ball_time_ = now_time;
          }
        } else if (dt > kBallStillS) {
          ball_vx_filt_ = 0.0;
          ball_vy_filt_ = 0.0;
        }
      }
      const double ball_vx = ball_vx_filt_;
      const double ball_vy = ball_vy_filt_;

      // Distâncias e controle de posse
      double min_ally_dist = std::numeric_limits<double>::infinity();
      double min_enemy_dist = std::numeric_limits<double>::infinity();
      double attacker_ball_dist = std::numeric_limits<double>::infinity();
      double defender_ball_dist = std::numeric_limits<double>::infinity();

      for (const auto& ally : gd->robots.allies) {
        const double dist = std::hypot(ally.x - ball_x, ally.y - ball_y);
        min_ally_dist = std::min(min_ally_dist, dist);

        if (ally.id == attacker_id_) {
          attacker_ball_dist = dist;
        }
        if (ally.id == self_id) {
          defender_ball_dist = dist;
        }
      }

      for (const auto& enemy : gd->robots.enemies) {
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

      double ball_age_ms = (now_time - last_game_data_time_).nanoseconds() / 1e6;
      if (ball_age_ms < 0.0) ball_age_ms = 0.0;
      blackboard_->set("ball_age_ms", ball_age_ms);
    }
  }

  void attachBranchObserver()
  {
    branch_subscriptions_.clear();
    root_branches_.clear();
    active_branch_.clear();
    tick_selected_branch_.clear();

    BT::TreeNode * root = tree_.rootNode();
    while (auto * dec = dynamic_cast<BT::DecoratorNode *>(root)) {
      root = dec->child();
    }

    auto * ctrl = dynamic_cast<BT::ControlNode *>(root);
    if (!ctrl) {
      return;
    }

    for (BT::TreeNode * child : ctrl->children()) {
      if (!child) {
        continue;
      }
      root_branches_.push_back(child);
      branch_subscriptions_.push_back(
        child->subscribeToStatusChange(
          [this](
            BT::TimePoint,
            const BT::TreeNode & node,
            BT::NodeStatus,
            BT::NodeStatus status)
          {
            if (status == BT::NodeStatus::SUCCESS) {
              tick_selected_branch_ = node.name();
            }
          }));
    }
  }

  static std::string phaseIdToString(uint8_t phase)
  {
    using oxebots_interfaces::msg::GameState;
    switch (phase) {
      case GameState::PHASE_HALT: return "HALT";
      case GameState::PHASE_TIMEOUT: return "TIMEOUT";
      case GameState::PHASE_STOP: return "STOP";
      case GameState::PHASE_BALL_PLACEMENT: return "BALL_PLACEMENT";
      case GameState::PHASE_PREPARE_KICKOFF: return "PREPARE_KICKOFF";
      case GameState::PHASE_KICKOFF: return "KICKOFF";
      case GameState::PHASE_PREPARE_PENALTY: return "PREPARE_PENALTY";
      case GameState::PHASE_PENALTY: return "PENALTY";
      case GameState::PHASE_FREE_KICK: return "FREE_KICK";
      case GameState::PHASE_RUN: return "RUN";
      default: return "UNKNOWN";
    }
  }

  std::string currentPhaseString() const
  {
    int phase = -1;
    if (blackboard_ && blackboard_->get("phase", phase) && phase >= 0 && phase <= 255) {
      return phaseIdToString(static_cast<uint8_t>(phase));
    }
    return "UNKNOWN";
  }

  std::string resolveTransitionReason(
    const std::string & prev_branch,
    const std::string & new_branch,
    const std::string & phase_str,
    const std::string & role_str) const
  {
    if (new_branch == "SafetyGuard") {
      double pose_age = 1e9;
      double ball_age = 1e9;
      double gc_age = 1e9;
      double max_pose = max_pose_ms_;
      double max_ball = max_ball_ms_;
      double max_gc = max_gc_ms_;
      bool req_gc = require_gc_;
      if (blackboard_) {
        (void)blackboard_->get("pose_age_ms", pose_age);
        (void)blackboard_->get("ball_age_ms", ball_age);
        (void)blackboard_->get("gc_age_ms", gc_age);
        (void)blackboard_->get("max_pose_ms", max_pose);
        (void)blackboard_->get("max_ball_ms", max_ball);
        (void)blackboard_->get("max_gc_ms", max_gc);
        (void)blackboard_->get("require_gc", req_gc);
      }
      if (pose_age > max_pose) {
        return "stale_pose";
      }
      if (ball_age > max_ball) {
        return "stale_ball";
      }
      if (req_gc && gc_age > max_gc) {
        return "stale_gc";
      }
      return "stale_data";
    }

    if (new_branch == "GoalkeeperRole" ||
        new_branch == "AttackerRole" ||
        new_branch == "DefenderTree" ||
        new_branch == "DefenderRole")
    {
      return "role=" + role_str;
    }

    if (new_branch == "Halt") {
      return "fallback_halt";
    }

    if (prev_branch == "SafetyGuard") {
      return "data_fresh,phase=" + phase_str;
    }

    return "phase=" + phase_str;
  }

  bool is_yellow_;
  bool invert_sides_;
  int robot_id_;
  uint32_t goalkeeper_id_{oxebots_interfaces::msg::RoleAssignment::NO_ROBOT};
  uint32_t attacker_id_{oxebots_interfaces::msg::RoleAssignment::NO_ROBOT};
  uint32_t defender_id_{oxebots_interfaces::msg::RoleAssignment::NO_ROBOT};
  uint32_t kicker_id_{oxebots_interfaces::msg::RoleAssignment::NO_ROBOT};

  double possession_distance_mm_{220.0};
  bool require_gc_{true};
  double max_gc_ms_{500.0};
  double max_pose_ms_{150.0};
  double max_ball_ms_{300.0};

  bool has_last_ball_{false};
  Pose2D last_ball_{0.0, 0.0, 0.0};
  rclcpp::Time last_ball_time_;
  double ball_vx_filt_{0.0};
  double ball_vy_filt_{0.0};

  std::atomic<bool> has_data_{false};
  std::atomic<bool> has_geometry_{false};
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> publisher_;
  BT::Blackboard::Ptr blackboard_;
  rclcpp::Subscription<oxebots_interfaces::msg::RoleAssignment>::SharedPtr role_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameState>::SharedPtr game_state_sub_;
  rclcpp::Subscription<ssl_league_msgs::msg::Referee>::SharedPtr gc_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::SSLGeometryData>::SharedPtr geometry_sub_;
  rclcpp::Publisher<oxebots_interfaces::msg::RobotMotionOverride>::SharedPtr override_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_branch_pub_;
  std::vector<BT::TreeNode::StatusChangeSubscriber> branch_subscriptions_;
  std::vector<BT::TreeNode *> root_branches_;
  std::string active_branch_;
  std::string tick_selected_branch_;

  // Buffer de mensagens e mutex para sincronização com a thread de tick
  std::mutex data_mutex_;
  oxebots_interfaces::msg::GameState::SharedPtr last_game_state_;
  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  rclcpp::Time last_game_data_time_;
  oxebots_interfaces::msg::SSLGeometryData::SharedPtr last_geometry_;
  oxebots_interfaces::msg::RoleAssignment::SharedPtr last_role_assignment_;
  ssl_league_msgs::msg::Referee::SharedPtr last_referee_;
};

#ifndef STRATEGY_NODE_NO_MAIN
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
#endif
