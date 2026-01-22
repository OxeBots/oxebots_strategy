#include "oxebots_strategy/rrt_star_planner.hpp"
#include <algorithm>
#include <iostream>

namespace oxebots_strategy
{

RRTStarPlanner::RRTStarPlanner(
    const Point& start, const Point& goal, const std::vector<Obstacle>& obstacles,
    double step_size, double goal_bias, double search_radius, int max_iterations,
    double robot_radius, double safety_margin)
    : start_(start), goal_(goal), obstacles_(obstacles),
      step_size_(step_size), goal_bias_(goal_bias), search_radius_(search_radius),
      max_iterations_(max_iterations), robot_radius_(robot_radius), safety_margin_(safety_margin),
      random_engine_(std::random_device{}()),
      uniform_dist_(0.0, 1.0)
{
    tree_.reserve(max_iterations_ + 1);
    tree_.push_back({start, -1, 0.0});
}

double distance_sq(const Point& p1, const Point& p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

double distance(const Point& p1, const Point& p2) {
    return std::sqrt(distance_sq(p1, p2));
}

std::vector<Point> RRTStarPlanner::plan_path() {
    for (int i = 0; i < max_iterations_; ++i) {
        Point random_point;
        if (uniform_dist_(random_engine_) < goal_bias_) {
            random_point = goal_;
        } else {
            
            random_point = {
                uniform_dist_(random_engine_) * 4.5 - 2.25, // X de -2.25 a 2.25
                uniform_dist_(random_engine_) * 3.0 - 1.5  // Y de -1.5 a 1.5
            };
        }

        int nearest_node_idx = get_nearest_node_index(random_point);
        Point new_point = steer(tree_[nearest_node_idx].pos, random_point);

        if (!is_collision(tree_[nearest_node_idx].pos, new_point)) {
            Node new_node = {new_point, nearest_node_idx, 0.0};
            tree_.push_back(new_node);
            int new_node_idx = tree_.size() - 1;

            choose_parent(new_node_idx, nearest_node_idx);
            rewire(new_node_idx);

            if (distance_sq(new_point, goal_) < step_size_ * step_size_) {
                if (!is_collision(new_point, goal_)) {
                    tree_.push_back({goal_, new_node_idx, tree_[new_node_idx].cost + distance(new_point, goal_)});
                    auto final_path = extract_path(tree_.size() - 1);
                    // auto pruned_path = prune_path(final_path);
                    // return smooth_path_bezier(pruned_path);
                    return final_path;
                }
            }
        }
    }

    // Return empty path if no solution found
    return {};
}

Point RRTStarPlanner::steer(const Point& from, const Point& to) {
    double dist = distance(from, to);
    if (dist < step_size_) {
        return to;
    }
    return {from.x + (to.x - from.x) / dist * step_size_,
            from.y + (to.y - from.y) / dist * step_size_};
}

int RRTStarPlanner::get_nearest_node_index(const Point& point) {
    int nearest_idx = -1;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < tree_.size(); ++i) {
        double d_sq = distance_sq(tree_[i].pos, point);
        if (d_sq < min_dist_sq) {
            min_dist_sq = d_sq;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

bool RRTStarPlanner::is_collision(const Point& p1, const Point& p2) {
    for (const auto& obs : obstacles_) {
        // Simple line-circle collision check
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double a = dx * dx + dy * dy;
        double b = 2 * (dx * (p1.x - obs.center.x) + dy * (p1.y - obs.center.y));
        double safe_radius = obs.radius + robot_radius_ + safety_margin_;
        double c = (p1.x - obs.center.x) * (p1.x - obs.center.x) +
                   (p1.y - obs.center.y) * (p1.y - obs.center.y) -
                   safe_radius * safe_radius;
        
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) continue;

        double t1 = (-b - std::sqrt(discriminant)) / (2 * a);
        double t2 = (-b + std::sqrt(discriminant)) / (2 * a);

        if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) {
            return true; // Collision
        }
    }
    return false; // No collision
}

void RRTStarPlanner::choose_parent(int new_node_idx, int nearest_node_idx) {
    Node& new_node = tree_[new_node_idx];
    new_node.parent_index = nearest_node_idx;
    new_node.cost = tree_[nearest_node_idx].cost + distance(new_node.pos, tree_[nearest_node_idx].pos);

    for (size_t i = 0; i < tree_.size(); ++i) {
        if (i == (size_t)new_node_idx) continue;
        
        const Node& neighbor = tree_[i];
        if (distance_sq(new_node.pos, neighbor.pos) < search_radius_ * search_radius_) {
            double new_cost = neighbor.cost + distance(new_node.pos, neighbor.pos);
            if (new_cost < new_node.cost && !is_collision(neighbor.pos, new_node.pos)) {
                new_node.parent_index = i;
                new_node.cost = new_cost;
            }
        }
    }
}

void RRTStarPlanner::rewire(int new_node_idx) {
    const Node& new_node = tree_[new_node_idx];
    for (size_t i = 0; i < tree_.size(); ++i) {
        if (i == (size_t)new_node_idx) continue;

        Node& neighbor = tree_[i];
        if (distance_sq(new_node.pos, neighbor.pos) < search_radius_ * search_radius_) {
            double cost_via_new_node = new_node.cost + distance(new_node.pos, neighbor.pos);
            if (cost_via_new_node < neighbor.cost && !is_collision(new_node.pos, neighbor.pos)) {
                neighbor.parent_index = new_node_idx;
                neighbor.cost = cost_via_new_node;
            }
        }
    }
}

std::vector<Point> RRTStarPlanner::extract_path(int goal_node_index) {
    std::vector<Point> path;
    int current_idx = goal_node_index;
    while (current_idx != -1) {
        path.push_back(tree_[current_idx].pos);
        current_idx = tree_[current_idx].parent_index;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Point> RRTStarPlanner::prune_path(const std::vector<Point>& path) {
    if (path.size() < 3) return path;

    std::vector<Point> pruned_path;
    pruned_path.push_back(path.front());

    size_t current_idx = 0;
    while (current_idx < path.size() - 1) {
        size_t next_idx = current_idx + 1;
        for (size_t i = path.size() - 1; i > next_idx; --i) {
            if (!is_collision(path[current_idx], path[i])) {
                next_idx = i;
                break;
            }
        }
        pruned_path.push_back(path[next_idx]);
        current_idx = next_idx;
    }
    return pruned_path;
}

std::vector<Point> RRTStarPlanner::smooth_path_bezier(const std::vector<Point>& path) {
    if (path.size() < 3) return path;

    std::vector<Point> smoothed_path;
    smoothed_path.push_back(path.front());

    for (size_t i = 0; i < path.size() - 2; ++i) {
        Point p0 = path[i];
        Point p1 = path[i+1];
        Point p2 = path[i+2];

        // Check distance between p0 and p2
        if (distance_sq(p0, p2) < step_size_ * step_size_) { // If too close, just add the middle point
            smoothed_path.push_back(p1);
        } else {
            // For segments between points, generate bezier curve points
            for (double t = 0.01; t <= 1.0; t += 0.05) { // 5% increment for curve points
                double one_minus_t = 1.0 - t;
                double x = one_minus_t * one_minus_t * p0.x + 2 * one_minus_t * t * p1.x + t * t * p2.x;
                double y = one_minus_t * one_minus_t * p0.y + 2 * one_minus_t * t * p1.y + t * t * p2.y;
                smoothed_path.push_back({x, y});
            }
        }
    }

    smoothed_path.push_back(path.back());
    return smoothed_path;
}

} // namespace oxebots_strategy
