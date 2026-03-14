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

#pragma once

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "rclcpp/rclcpp.hpp"

namespace planning
{

/**
 * @enum Tag
 * @brief Represents the state of a node in the D* search process.
 */
enum class Tag
{
    NEW,    ///< Node has never been visited.
    OPEN,   ///< Node is currently in the priority queue (Open List).
    CLOSED  ///< Node has been processed and removed from the Open List.
};

/**
 * @enum HeuristicType
 * @brief Defines the heuristic function used for cost estimation.
 */
enum class HeuristicType
{
    EUCLIDEAN,  ///< Straight line distance (hypot).
    MANHATTAN,  ///< L1 distance (sum of absolute differences).
    DIAGONAL    ///< Octile distance (allows diagonal movement).
};

/**
 * @enum PlannerStatus
 * @brief Result status of a planning request.
 */
enum class PlannerStatus
{
    SUCCESS,            ///< Path found successfully.
    FAILURE,            ///< Failed to find a path (unreachable or timeout).
    INVALID_START_GOAL  ///< Start or Goal inputs are invalid/unset.
};

/**
 * @struct GridCell
 * @brief Represents a discrete coordinate on the occupancy grid.
 */
struct GridCell
{
    int x = 0;  ///< X grid coordinate.
    int y = 0;  ///< Y grid coordinate.

    bool operator==(const GridCell & other) const { return x == other.x && y == other.y; }
    bool operator!=(const GridCell & other) const { return !(*this == other); }
};

/**
 * @struct DStarNode
 * @brief Represents a node in the D* graph.
 * @details Designed to be stored in a flat vector to avoid dynamic allocation overhead.
 */
struct DStarNode
{
    GridCell cell;                 ///< The grid coordinates of this node.
    DStarNode * parent = nullptr;  ///< Pointer to the parent node in the path.
    Tag tag = Tag::NEW;            ///< Current search tag (NEW, OPEN, CLOSED).
    double h = 0.0;                ///< Cost from this node to the Goal (g-value in standard D* Lite).
    double k = 0.0;                ///< Key value (minimum of current h and previous h), used for priority.

    /**
     * @brief Resets the node state for a fresh planning cycle if needed.
     * @note In D*, we typically don't reset everything to preserve search history, but this is used
     * when the map changes drastically or on initialization.
     */
    void reset()
    {
        parent = nullptr;
        tag = Tag::NEW;
        h = 0.0;
        k = 0.0;
    }
};

/**
 * @struct PlannerConfig
 * @brief Configuration parameters for the planner.
 */
struct PlannerConfig
{
    double robot_safety_radius = 0.15;  ///< Radius to pad around obstacles (in meters).
    int occupancy_threshold = 80;       ///< Map cell value (0-100) to consider as an obstacle.
    int max_expansions = 100000;        ///< Maximum number of node expansions before giving up.
    int max_replan_steps = 50000;       ///< Maximum expansions allowed during incremental replanning.
    double sensor_range = 1.0;          ///< Range around the robot to check for map changes (meters).
    HeuristicType heuristic_type = HeuristicType::EUCLIDEAN;  ///< Heuristic function to use.
};

/**
 * @struct PlannerMetrics
 * @brief runtime performance metrics for debugging.
 */
struct PlannerMetrics
{
    std::chrono::microseconds planning_time{0};  ///< Time taken for the last plan/replan.
    int total_expansions = 0;                    ///< Count of nodes processed.
    int path_length = 0;                         ///< Number of waypoints in the resulting path.
    double path_cost = 0.0;                      ///< Total accumulated cost of the path.

    void reset()
    {
        planning_time = std::chrono::microseconds(0);
        total_expansions = 0;
        path_length = 0;
        path_cost = 0.0;
    }
};

/**
 * @class DStarPlanner
 * @brief Core implementation of the D* pathfinding algorithm.
 * @details This class handles map management, cost calculations, and the D* search loop. It uses a
 * flattened node array for cache efficiency and performance.
 */
class DStarPlanner
{
   public:
    /**
     * @brief Constructor.
     * @param resolution Map resolution in meters per cell.
     * @param config Configuration struct.
     * @param logger ROS 2 logger for debug output.
     */
    DStarPlanner(double resolution, const PlannerConfig & config, rclcpp::Logger logger);
    ~DStarPlanner() = default;

    /**
     * @brief Updates the internal occupancy grid.
     * @details Reallocates the graph only if map dimensions change.
     * @param grid Shared pointer to the new occupancy grid message.
     */
    void setOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr & grid);

    /**
     * @brief Updates positions of dynamic obstacles (allies).
     * @param allies List of (x, y) coordinates of ally robots.
     */
    void setAllyPositions(const std::vector<std::pair<float, float>> & allies);

    /**
     * @brief Initializes a new search from Start to Goal.
     * @details Resets the open list and inserts the goal node (search is backward).
     * @param start Start grid coordinates.
     * @param goal Goal grid coordinates.
     */
    void initialize(const GridCell & start, const GridCell & goal);

    /**
     * @brief Executes the planning loop.
     * @return PlannerStatus::SUCCESS if path found, FAILURE otherwise.
     */
    PlannerStatus plan();

    /**
     * @brief Incrementally updates map costs and repairs the path.
     * @details This is the core "D*" feature: handling dynamic map changes without full replanning.
     * @param current_pos Current robot grid position.
     */
    void updateMapAndReplan(const GridCell & current_pos);

    /**
     * @brief Reconstructs the path from Start to Goal by following parent pointers.
     * @param start Start grid coordinates.
     * @param map_origin_x Map origin X in world coordinates.
     * @param map_origin_y Map origin Y in world coordinates.
     * @return nav_msgs::msg::Path The resulting path in world coordinates.
     */
    nav_msgs::msg::Path reconstructPath(const GridCell & start, double map_origin_x, double map_origin_y);


    /**
     * @brief Converts World coordinates to Grid coordinates.
     */
    GridCell worldToGrid(double wx, double wy, double origin_x, double origin_y) const;

    /**
     * @brief Converts Grid coordinates to World coordinates.
     */
    geometry_msgs::msg::PoseStamped gridToWorld(const GridCell & gc, double origin_x, double origin_y) const;

    std::optional<GridCell> getGoal() const { return goal_cell_; }
    const PlannerMetrics & getMetrics() const { return metrics_; }
    void resetMetrics() { metrics_.reset(); }

   private:
    /**
     * @struct OpenListEntry
     * @brief Entry for the priority queue (Open List).
     */
    struct OpenListEntry
    {
        double sort_key;   ///< Primary sort key (f-cost).
        double raw_k;      ///< Secondary sort key (minimum path cost).
        DStarNode * node;  ///< Pointer to the actual node.

        /**
         * @brief Comparator for priority queue (min-heap).
         * @return true if 'this' has higher cost than 'other'.
         */
        bool operator>(const OpenListEntry & other) const;
    };

    // Internal Logic
    double calculateHeuristic(const GridCell & a, const GridCell & b) const;
    double getSiteCost(const GridCell & u) const;
    double getTraversalCost(DStarNode * n1, DStarNode * n2) const;
    std::vector<DStarNode *> getNeighbors(DStarNode * u);
    DStarNode * getNode(int x, int y);

    /**
     * @brief Inserts or updates a node in the Open List.
     * @param node The node to insert.
     * @param h_new The new h-value (path cost).
     */
    void insert(DStarNode * node, double h_new);

    /**
     * @brief Processes the top node in the Open List.
     * @details Expands the node and propagates cost changes to neighbors.
     * @return The raw_k value of the processed node, or -1.0 if list empty.
     */
    double processState();

    /**
     * @brief Marks a node for re-evaluation when cost changes.
     */
    void modifyCost(DStarNode * node);

    // Data
    double resolution_;
    PlannerConfig config_;
    rclcpp::Logger logger_;
    PlannerMetrics metrics_;

    // Map & Graph (Flat vector for performance)
    nav_msgs::msg::OccupancyGrid::SharedPtr current_grid_;
    std::vector<DStarNode> grid_nodes_;  ///< Flat storage for all graph nodes.
    std::vector<bool> dynamic_obstacle_map_; ///< Fast lookup for dynamic obstacles.
    int grid_width_ = 0;
    int grid_height_ = 0;

    // Dynamic Obstacles
    std::vector<GridCell> dynamic_obstacles_;

    // Algorithm State
    std::priority_queue<OpenListEntry, std::vector<OpenListEntry>, std::greater<OpenListEntry>> open_list_;
    std::optional<GridCell> start_cell_;
    std::optional<GridCell> goal_cell_;
    mutable std::mutex data_mutex_;
};

/**
 * @class DStarPlannerNode
 * @brief ROS 2 Node wrapper for the DStarPlanner.
 * @details Handles ROS communication (subscribers/publishers) and transformation of data.
 */
class DStarPlannerNode : public rclcpp::Node
{
   public:
    DStarPlannerNode();

   private:
    void game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
    void goal_callback(const oxebots_interfaces::msg::RobotGoal::SharedPtr msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /**
     * @brief Timer callback to trigger planning iteration.
     * @details Checks conditions, runs planner, and publishes path.
     */
    void plan_and_publish();

    // ROS Handles
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rviz_path_pub_;
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr planning_timer_;

    // Components
    std::unique_ptr<planning::DStarPlanner> planner_;
    nav_msgs::msg::OccupancyGrid::SharedPtr last_map_data_;
    std::mutex node_mutex_;

    // State
    std::optional<oxebots_interfaces::msg::RobotGoal::SharedPtr> target_goal_msg_;
    float current_x_ = 0.0f;
    float current_y_ = 0.0f;
    int robot_id_ = 0;
    int consecutive_failures_ = 0;
};
}  // namespace planning
