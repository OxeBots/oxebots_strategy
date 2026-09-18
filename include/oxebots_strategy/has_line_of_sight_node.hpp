#pragma once
#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include "oxebots_interfaces/msg/game_data.hpp"
#include <cmath>
#include <vector>
#include <mutex>

namespace oxebots_strategy {

class HasLineOfSightNode : public BT::ConditionNode
{
public:
    HasLineOfSightNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::ConditionNode(name, config), node_(node)
    {
        game_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
            "game_data", 10, [this](const oxebots_interfaces::msg::GameData::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_enemies_ = msg->robots.enemies;
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("origin_x"),
            BT::InputPort<double>("origin_y"),
            BT::InputPort<double>("target_x"),
            BT::InputPort<double>("target_y")
        };
    }

    BT::NodeStatus tick() override
    {
        double ox, oy, tx, ty;
        if (!getInput("origin_x", ox) || !getInput("origin_y", oy) || 
            !getInput("target_x", tx) || !getInput("target_y", ty)) {
            return BT::NodeStatus::FAILURE;
        }

        std::vector<oxebots_interfaces::msg::RobotGameData> enemies_copy;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            enemies_copy = latest_enemies_;
        }

        // Raio do robô + tolerância de segurança em mm (ex: 90mm de raio + 30mm)
        const double threshold = 120.0; 

        // Vetor da linha de visão
        double dx = tx - ox;
        double dy = ty - oy;
        double length_sq = dx * dx + dy * dy;

        if (length_sq == 0) return BT::NodeStatus::SUCCESS;

        for (const auto& enemy : enemies_copy) {
            // Projeção escalar do oponente na linha de visão (clamp entre 0 e 1)
            double t = ((enemy.x - ox) * dx + (enemy.y - oy) * dy) / length_sq;
            t = std::max(0.0, std::min(1.0, t));

            // Ponto mais próximo na reta
            double closest_x = ox + t * dx;
            double closest_y = oy + t * dy;

            // Distância ao quadrado entre o oponente e o ponto mais próximo
            double dist_sq = (enemy.x - closest_x) * (enemy.x - closest_x) + 
                             (enemy.y - closest_y) * (enemy.y - closest_y);

            if (dist_sq < (threshold * threshold)) {
                return BT::NodeStatus::FAILURE; // Caminho bloqueado
            }
        }

        return BT::NodeStatus::SUCCESS; // Caminho livre!
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_sub_;
    std::vector<oxebots_interfaces::msg::RobotGameData> latest_enemies_;
    std::mutex mutex_;
};

} // namespace oxebots_strategy