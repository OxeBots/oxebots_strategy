#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"

#include <optional>
#include <vector>
#include <mutex>
#include <cmath>

namespace movement {
    // Definição da estrutura Coordinate
    struct Coordinate {
        float x = 0.0f;
        float y = 0.0f;
        float orientation = 0.0f;
    };

    // Definição da classe PotentialField
    class PotentialField {
    public:
        std::vector<double> calculate(const Coordinate& current_pos, const Coordinate& target_pos, const std::vector<Coordinate>& obstacles) {
            double attractive_gain = 5.0;
            double repulsive_gain = 2.0;
            double obstacle_radius = 180.0; // Raio do obstáculo em mm

            double attractive_force_x = attractive_gain * (target_pos.x - current_pos.x);
            double attractive_force_y = attractive_gain * (target_pos.y - current_pos.y);

            double repulsive_force_x = 0.0;
            double repulsive_force_y = 0.0;

            for (const auto& obs : obstacles) {
                double dist = std::sqrt(std::pow(current_pos.x - obs.x, 2) + std::pow(current_pos.y - obs.y, 2));
                if (dist < obstacle_radius && dist > 1.0) { // Evita divisão por zero
                    double angle = std::atan2(current_pos.y - obs.y, current_pos.x - obs.x);
                    double force = repulsive_gain * (1.0 / dist - 1.0 / obstacle_radius) / std::pow(dist, 2);
                    repulsive_force_x += force * std::cos(angle);
                    repulsive_force_y += force * std::sin(angle);
                }
            }

            double total_force_x = attractive_force_x + repulsive_force_x;
            double total_force_y = attractive_force_y + repulsive_force_y;
            double target_angle = std::atan2(total_force_y, total_force_x);

            return {total_force_x, total_force_y, target_angle};
        }
    };

    inline double calculateDistance(const Coordinate& a, const Coordinate& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }
}

class PotentialFieldNode : public rclcpp::Node {
public:
    PotentialFieldNode();

private:
    void game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void calculate_and_move();

    rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex data_mutex_;
    std::optional<movement::Coordinate> target_pos_;
    oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
    unsigned int robot_id_ = 0; // ID do robô a ser controlado
    bool game_data_received_ = false;
};
