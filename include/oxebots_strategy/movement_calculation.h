#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "nav_msgs/msg/path.hpp"

#include <optional>
#include <vector>
#include <mutex>
#include <cmath>

namespace movement {
    struct Coordinate {
        float x = 0.0f;
        float y = 0.0f;
        float orientation = 0.0f;
    };

    inline double calculateDistance(const Coordinate& a, const Coordinate& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }
}

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode();

private:
    void game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
    void goal_callback(const oxebots_interfaces::msg::RobotGoal::SharedPtr msg);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void calculate_and_move();

    rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex data_mutex_;
    std::optional<movement::Coordinate> target_goal_;
    nav_msgs::msg::Path::SharedPtr last_path_;
    std::optional<double> target_w_; 

    oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
    int robot_id_ = 0; 
    bool game_data_received_ = false;

    bool is_yellow_team_;
};
