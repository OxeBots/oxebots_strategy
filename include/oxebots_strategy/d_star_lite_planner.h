#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "nav_msgs/msg/occupancy_grid.hpp"    
#include "nav_msgs/msg/path.hpp"              
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"

#include <memory>
#include <optional>
#include <vector>
#include <mutex>
#include <cmath>
#include <unordered_map>
#include <queue>

namespace movement {
    struct Coordinate {
        float x = 0.0f;
        float y = 0.0f;
        float orientation = 0.0f;
    };
}

namespace planning {

    struct GridCell {
        int x; 
        int y; 

        bool operator==(const GridCell& other) const { return x == other.x && y == other.y; }
        bool operator!=(const GridCell& other) const { return !(*this == other); }
        bool operator<(const GridCell& other) const {
            return std::tie(x, y) < std::tie(other.x, other.y);
        }
    };
    
    struct GridCellHasher {
        std::size_t operator()(const GridCell& p) const {
            return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 16);
        }
    };

    struct DStarNode {
        double g = std::numeric_limits<double>::infinity();
        double rhs = std::numeric_limits<double>::infinity();
        std::pair<double, double> key = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        GridCell cell;
        GridCell successor = {-1, -1};
    };

    struct KeyComparator {
        bool operator()(const DStarNode* a, const DStarNode* b) const {
            if (std::abs(a->key.first - b->key.first) > 1e-6) {
                return a->key.first > b->key.first; 
            }
            return a->key.second > b->key.second; 
        }
    };

    class DStarLitePlanner {
    public:
        DStarLitePlanner(double resolution);
        ~DStarLitePlanner();
        
        void initialize(const GridCell& start, const GridCell& goal);
        bool computePath();
        void setStart(const GridCell& start);
        nav_msgs::msg::Path reconstructPath(const GridCell& start, double map_origin_x, double map_origin_y);
        GridCell worldToGrid(double wx, double wy, double map_origin_x, double map_origin_y) const;
        geometry_msgs::msg::PoseStamped gridToWorld(const GridCell& gc, double map_origin_x, double map_origin_y) const;
        void setOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr& grid);
        void setAllyPositions(const std::vector<movement::Coordinate>& allies); 
        double getCost(const GridCell& u, const GridCell& v) const;
        void checkAndModifyCosts(const GridCell& current_start);
        std::optional<GridCell> getGoal() const { return goal_cell_; }

    private:
        std::unordered_map<GridCell, DStarNode*, GridCellHasher> nodes_;
        std::priority_queue<DStarNode*, std::vector<DStarNode*>, KeyComparator> open_list_;
        double k_m_ = 0.0;
        double resolution_;
        std::optional<GridCell> start_cell_;
        std::optional<GridCell> goal_cell_;
        std::vector<movement::Coordinate> ally_positions_;
        mutable std::mutex ally_mutex_; 
        double robot_safety_radius_ = 0.15;
        
        double calculateHeuristic(const GridCell& a, const GridCell& b) const;
        std::pair<double, double> calculateKey(const GridCell& u);
        void updateVertex(DStarNode* u);
        std::vector<GridCell> getNeighbors(const GridCell& u) const;
        DStarNode* getOrCreateNode(const GridCell& u);
        nav_msgs::msg::OccupancyGrid::SharedPtr current_grid_;
        mutable std::mutex grid_mutex_;
        std::unordered_map<GridCell, double, GridCellHasher> previous_costs_; 
    };

} // namespace planning

class DStarLitePlannerNode : public rclcpp::Node {
public:
    DStarLitePlannerNode();

private:
    void game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
    void goal_callback(const oxebots_interfaces::msg::RobotGoal::SharedPtr msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void plan_and_move();
    void follow_path_point_P(const geometry_msgs::msg::PoseStamped& next_point_pose);

    rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_; 
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<planning::DStarLitePlanner> planner_;
    nav_msgs::msg::OccupancyGrid::SharedPtr last_map_data_;
    std::mutex data_mutex_;
    std::optional<oxebots_interfaces::msg::RobotGoal::SharedPtr> target_goal_msg_;
    unsigned int robot_id_ = 0; 
    movement::Coordinate current_pos_world_; 
};