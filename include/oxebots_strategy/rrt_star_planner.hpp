#ifndef OXEBOTS_STRATEGY__RRT_STAR_PLANNER_HPP_
#define OXEBOTS_STRATEGY__RRT_STAR_PLANNER_HPP_

#include <vector>
#include <cmath>
#include <random>

namespace oxebots_strategy
{

struct Point {
    double x, y;
};

struct Node {
    Point pos;
    int parent_index;
    double cost;
};

struct Obstacle {
    Point center;
    double radius;
};

class RRTStarPlanner {
public:
    RRTStarPlanner(
        const Point& start, const Point& goal, const std::vector<Obstacle>& obstacles,
        double step_size = 0.3, double goal_bias = 0.3, double search_radius = 0.5,
        int max_iterations = 2000, double robot_radius = 0.09, double safety_margin = 0.05
    );

    std::vector<Point> plan_path();

private:
    Point steer(const Point& from, const Point& to);
    int get_nearest_node_index(const Point& point);
    bool is_collision(const Point& p1, const Point& p2);
    void choose_parent(int new_node_index, int nearest_node_index);
    void rewire(int new_node_index);
    std::vector<Point> extract_path(int goal_node_index);
    std::vector<Point> prune_path(const std::vector<Point>& path);
    std::vector<Point> smooth_path_bezier(const std::vector<Point>& path);

    Point start_;
    Point goal_;
    std::vector<Obstacle> obstacles_;
    std::vector<Node> tree_;
    
    double step_size_;
    double goal_bias_;
    double search_radius_;
    int max_iterations_;
    double robot_radius_;
    double safety_margin_;

    std::mt19937 random_engine_;
    std::uniform_real_distribution<> uniform_dist_;
};

} // namespace oxebots_strategy

#endif // OXEBOTS_STRATEGY__RRT_STAR_PLANNER_HPP_
