#include "oxebots_strategy/defender_nodes.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/role_assignment.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <fstream>
#include <limits>
#include <string>
#include <thread>

class DefenderStrategyNode : public rclcpp::Node
{
public:
    DefenderStrategyNode()
        : rclcpp::Node("defender_strategy_node")
    {
        this->declare_parameter<std::string>("bt_xml_path", "defender_tree.xml");
        this->declare_parameter<bool>("is_yellow_team", false);
        this->declare_parameter<int>("robot_id", 2);
        this->declare_parameter<double>("execution_rate", 60.0);
        this->declare_parameter<double>("possession_distance", 220.0);
        this->declare_parameter<double>("goal_x_threshold", 700.0);
        this->declare_parameter<double>("goal_band_half_width", 900.0);
        this->declare_parameter<double>("moving_towards_goal_velocity_threshold", 120.0);

        is_yellow_ = this->get_parameter("is_yellow_team").as_bool();
        robot_id_ = this->get_parameter("robot_id").as_int();
        execution_rate_hz_ = this->get_parameter("execution_rate").as_double();
        possession_distance_mm_ = this->get_parameter("possession_distance").as_double();
        goal_x_threshold_mm_ = this->get_parameter("goal_x_threshold").as_double();
        goal_band_half_width_mm_ = this->get_parameter("goal_band_half_width").as_double();
        moving_towards_goal_velocity_threshold_mm_s_ =
            this->get_parameter("moving_towards_goal_velocity_threshold").as_double();

        blackboard_ = BT::Blackboard::create();
        setupBlackboard();

        role_sub_ = this->create_subscription<oxebots_interfaces::msg::RoleAssignment>(
            "/role_assignment", 10,
            std::bind(&DefenderStrategyNode::roleCallback, this, std::placeholders::_1));

        game_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
            "game_data", 10,
            std::bind(&DefenderStrategyNode::gameCallback, this, std::placeholders::_1));
    }

    bool init()
    {
        try {
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("oxebots_strategy");
            std::string tree_path = this->get_parameter("bt_xml_path").as_string();

            if (tree_path.empty()) {
                tree_path = package_share_directory + "/behavior_trees/defender_tree.xml";
            } else if (tree_path.find("/") == std::string::npos) {
                tree_path = package_share_directory + "/behavior_trees/" + tree_path;
            }

            std::ifstream file(tree_path);
            if (!file.good()) {
                RCLCPP_ERROR(this->get_logger(), "ARQUIVO NÃO ENCONTRADO: %s", tree_path.c_str());
                return false;
            }
            file.close();

            configureDefenderController(shared_from_this(), static_cast<uint32_t>(robot_id_));

            factory_.registerNodeType<IsBallInOpponentField>("IsBallInOpponentField");
            factory_.registerNodeType<IsOpponentControllingBall>("IsOpponentControllingBall");
            factory_.registerNodeType<IsBallNearGoal>("IsBallNearGoal");
            factory_.registerNodeType<IsDefenderCloserThanAttacker>("IsDefenderCloserThanAttacker");
            factory_.registerNodeType<IsBallMovingTowardsGoal>("IsBallMovingTowardsGoal");

            factory_.registerNodeType<GoToSafe>("GoToSafe");
            factory_.registerNodeType<BlockShot>("BlockShot");
            factory_.registerNodeType<PressureBall>("PressureBall");
            factory_.registerNodeType<BlockAngle>("BlockAngle");
            factory_.registerNodeType<InterceptBall>("InterceptBall");
            factory_.registerNodeType<DefensivePosition>("DefensivePosition");

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

            factory_.registerSimpleCondition("IsRobotId", [&](BT::TreeNode& node) {
                uint32_t target_id, my_id;
                if (!node.getInput("robot_id", target_id)) return BT::NodeStatus::FAILURE;
                if (!blackboard_->get("robot_id", my_id)) return BT::NodeStatus::FAILURE;
                return (my_id == target_id) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            }, { BT::InputPort<uint32_t>("robot_id") });

            tree_ = factory_.createTreeFromFile(tree_path, blackboard_);

            RCLCPP_INFO(this->get_logger(), "Defender BT carregada: %s", tree_path.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "EXCEÇÃO NO INIT: %s", e.what());
            return false;
        }
    }

    void run()
    {
        if (execution_rate_hz_ <= 0.1) {
            execution_rate_hz_ = 60.0;
        }

        rclcpp::Rate rate(execution_rate_hz_);

        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());

            if (has_game_data_) {
                try {
                    tree_.tickOnce();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Erro ao executar tick: %s", e.what());
                }
            } else {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Aguardando dados de jogo para executar defender tree...");
            }

            try {
                rate.sleep();
            } catch (...) {
                // Fallback if time jumps invalidate rclcpp::Rate sleep
                std::this_thread::sleep_for(std::chrono::milliseconds(
                    static_cast<int>(1000.0 / execution_rate_hz_)));
            }
        }
    }

private:
    static double distance(double x1, double y1, double x2, double y2)
    {
        return std::hypot(x1 - x2, y1 - y2);
    }

    void setupBlackboard()
    {
        const double my_goal_x = is_yellow_ ? 2200.0 : -2200.0;

        blackboard_->set("is_yellow", is_yellow_);
        blackboard_->set("robot_id", static_cast<uint32_t>(robot_id_));
        blackboard_->set("my_goal_x", my_goal_x);
        blackboard_->set("goal_x_threshold", goal_x_threshold_mm_);
        blackboard_->set("goal_band_half_width", goal_band_half_width_mm_);
        blackboard_->set("moving_towards_goal_velocity_threshold", moving_towards_goal_velocity_threshold_mm_s_);

        blackboard_->set("ball_x", 0.0);
        blackboard_->set("ball_y", 0.0);
        blackboard_->set("ball_vx", 0.0);
        blackboard_->set("ball_vy", 0.0);
        blackboard_->set("ball", Pose2D{0.0, 0.0, 0.0});
        blackboard_->set("ball_vel", Vector2D{0.0, 0.0});

        blackboard_->set("opponent_controlling", false);
        blackboard_->set("attacker_ball_dist", std::numeric_limits<double>::infinity());
        blackboard_->set("defender_ball_dist", std::numeric_limits<double>::infinity());

        blackboard_->set("attacker_id", attacker_id_);
        blackboard_->set("defender_id", defender_id_);
    }

    void roleCallback(const oxebots_interfaces::msg::RoleAssignment::SharedPtr msg)
    {
        attacker_id_ = msg->attacker_id;
        defender_id_ = msg->defender_id;

        blackboard_->set("attacker_id", attacker_id_);
        blackboard_->set("defender_id", defender_id_);
    }

    void gameCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
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
            const double dist = distance(ally.x, ally.y, ball_x, ball_y);
            min_ally_dist = std::min(min_ally_dist, dist);

            if (ally.id == attacker_id_) {
                attacker_ball_dist = dist;
            }
            if (ally.id == self_id) {
                defender_ball_dist = dist;
            }
        }

        for (const auto& enemy : msg->robots.enemies) {
            const double dist = distance(enemy.x, enemy.y, ball_x, ball_y);
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

        has_game_data_ = true;
    }

    bool is_yellow_{false};
    int robot_id_{2};
    uint32_t attacker_id_{1};
    uint32_t defender_id_{2};

    double execution_rate_hz_{60.0};
    double possession_distance_mm_{220.0};
    double goal_x_threshold_mm_{700.0};
    double goal_band_half_width_mm_{900.0};
    double moving_towards_goal_velocity_threshold_mm_s_{120.0};

    bool has_game_data_{false};
    bool has_last_ball_{false};
    Pose2D last_ball_{0.0, 0.0, 0.0};
    rclcpp::Time last_ball_time_;

    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;

    rclcpp::Subscription<oxebots_interfaces::msg::RoleAssignment>::SharedPtr role_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DefenderStrategyNode>();
    if (!node->init()) {
        RCLCPP_FATAL(node->get_logger(), "Falha crítica na inicialização do DefenderStrategyNode.");
        rclcpp::shutdown();
        return 1;
    }

    node->run();

    rclcpp::shutdown();
    return 0;
}