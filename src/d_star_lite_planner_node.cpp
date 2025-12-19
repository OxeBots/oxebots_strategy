#include "oxebots_strategy/d_star_lite_planner.h"

namespace {
// Normaliza ângulo para o intervalo [-PI, PI]
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
}

namespace planning {

DStarLitePlanner::DStarLitePlanner(double resolution) : resolution_(resolution) {}
DStarLitePlanner::~DStarLitePlanner() {
    for (auto& [key, val] : nodes_) delete val;
}

DStarNode* DStarLitePlanner::getOrCreateNode(const GridCell& u) {
    auto it = nodes_.find(u);
    if (it == nodes_.end()) {
        DStarNode* newNode = new DStarNode();
        newNode->cell = u;
        nodes_[u] = newNode;
        return newNode;
    }
    return it->second;
}

void DStarLitePlanner::setOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr& grid) {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    current_grid_ = grid;
}

void DStarLitePlanner::setAllyPositions(const std::vector<movement::Coordinate>& allies) {
    std::lock_guard<std::mutex> lock(ally_mutex_);
    ally_positions_ = allies;
}

double DStarLitePlanner::getCost(const GridCell& u, const GridCell& v) const {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    if (!current_grid_) return std::numeric_limits<double>::infinity();
    if (v.x < 0 || v.y < 0 || v.x >= (int)current_grid_->info.width || v.y >= (int)current_grid_->info.height)
        return std::numeric_limits<double>::infinity();

    int index = v.x + v.y * current_grid_->info.width;
    if (current_grid_->data[index] >= 80) return 1000.0;

    {
        std::lock_guard<std::mutex> lock_ally(ally_mutex_);
        double vx = current_grid_->info.origin.position.x + (v.x + 0.5) * resolution_;
        double vy = current_grid_->info.origin.position.y + (v.y + 0.5) * resolution_;
        for (const auto& ally : ally_positions_) {
            if (std::hypot(vx - ally.x, vy - ally.y) < robot_safety_radius_) return 500.0;
        }
    }
    return (std::abs(u.x - v.x) + std::abs(u.y - v.y) > 1) ? 1.414 : 1.0;
}

double DStarLitePlanner::calculateHeuristic(const GridCell& a, const GridCell& b) const {
    return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
}

std::pair<double, double> DStarLitePlanner::calculateKey(const GridCell& u) {
    DStarNode* node = getOrCreateNode(u);
    double h = calculateHeuristic(u, start_cell_.value());
    double min_v = std::min(node->g, node->rhs);
    return {min_v + h + k_m_, min_v};
}

void DStarLitePlanner::updateVertex(DStarNode* u) {
    if (u->cell != goal_cell_.value()) {
        double min_rhs = std::numeric_limits<double>::infinity();
        GridCell best_succ = {-1, -1};
        for (const auto& s_prime : getNeighbors(u->cell)) {
            double cost = getCost(u->cell, s_prime);
            double val = getOrCreateNode(s_prime)->g + cost;
            if (val < min_rhs) {
                min_rhs = val;
                best_succ = s_prime;
            }
        }
        u->rhs = min_rhs;
        u->successor = best_succ;
    }
    if (u->g != u->rhs) {
        u->key = calculateKey(u->cell);
        open_list_.push(u);
    }
}

bool DStarLitePlanner::computePath() {
    if (open_list_.empty()) return false;
    while (!open_list_.empty()) {
        DStarNode* u = open_list_.top();
        if (u->key < calculateKey(u->cell)) {
            u->key = calculateKey(u->cell);
            open_list_.pop();
            open_list_.push(u);
            continue;
        }
        if (u->key >= calculateKey(start_cell_.value()) && 
            getOrCreateNode(start_cell_.value())->g == getOrCreateNode(start_cell_.value())->rhs) break;
        
        open_list_.pop();
        if (u->g > u->rhs) {
            u->g = u->rhs;
            for (const auto& s : getNeighbors(u->cell)) updateVertex(getOrCreateNode(s));
        } else {
            u->g = std::numeric_limits<double>::infinity();
            updateVertex(u);
            for (const auto& s : getNeighbors(u->cell)) updateVertex(getOrCreateNode(s));
        }
    }
    return getOrCreateNode(start_cell_.value())->g != std::numeric_limits<double>::infinity();
}

void DStarLitePlanner::setStart(const GridCell& start) {
    if (start_cell_.has_value() && start_cell_.value() != start) {
        k_m_ += calculateHeuristic(start_cell_.value(), start);
        start_cell_ = start;
    }
}

void DStarLitePlanner::initialize(const GridCell& start, const GridCell& goal) {
    for (auto& [k, v] : nodes_) {
        delete v;
    }
    nodes_.clear();
    while(!open_list_.empty()) open_list_.pop();
    
    start_cell_ = start; 
    goal_cell_ = goal; 
    k_m_ = 0.0;
    
    DStarNode* goalNode = getOrCreateNode(goal);
    goalNode->rhs = 0.0; 
    goalNode->key = calculateKey(goal);
    open_list_.push(goalNode);
}

std::vector<GridCell> DStarLitePlanner::getNeighbors(const GridCell& u) const {
    std::vector<GridCell> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            neighbors.push_back({u.x + dx, u.y + dy});
        }
    }
    return neighbors;
}

nav_msgs::msg::Path DStarLitePlanner::reconstructPath(const GridCell& start, double ox, double oy) {
    nav_msgs::msg::Path path;
    GridCell curr = start;
    for (int i = 0; i < 500 && curr != goal_cell_.value(); ++i) {
        path.poses.push_back(gridToWorld(curr, ox, oy));
        if (getOrCreateNode(curr)->successor.x == -1) break;
        curr = getOrCreateNode(curr)->successor;
    }
    path.poses.push_back(gridToWorld(goal_cell_.value(), ox, oy));
    return path;
}

GridCell DStarLitePlanner::worldToGrid(double wx, double wy, double ox, double oy) const {
    return {(int)((wx - ox) / resolution_), (int)((wy - oy) / resolution_)};
}

geometry_msgs::msg::PoseStamped DStarLitePlanner::gridToWorld(const GridCell& gc, double ox, double oy) const {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = ox + (gc.x + 0.5) * resolution_;
    ps.pose.position.y = oy + (gc.y + 0.5) * resolution_;
    ps.pose.orientation.w = 1.0;
    return ps;
}

void DStarLitePlanner::checkAndModifyCosts(const GridCell& start) {
    for (const auto& v : getNeighbors(start)) {
        double cost = getCost(start, v);
        if (previous_costs_.count(v) && std::abs(previous_costs_[v] - cost) > 0.1) 
            updateVertex(getOrCreateNode(start));
        previous_costs_[v] = cost;
    }
}

} // namespace planning

// --- IMPLEMENTAÇÃO DO NÓ ROS 2 ---

DStarLitePlannerNode::DStarLitePlannerNode() : Node("d_star_lite_planner_node") {
    planner_ = std::make_unique<planning::DStarLitePlanner>(0.01);
    this->declare_parameter("robot_id", 0);
    this->get_parameter("robot_id", robot_id_);
    
    cmd_vel_pub_ = this->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/dstar_path", 10);
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", qos, 
        std::bind(&DStarLitePlannerNode::map_callback, this, std::placeholders::_1));
    
    game_data_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>("/game_data", 10, 
        std::bind(&DStarLitePlannerNode::game_data_callback, this, std::placeholders::_1));
    
    goal_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotGoal>("/robot_goal", 
        rclcpp::QoS(1).transient_local(), std::bind(&DStarLitePlannerNode::goal_callback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), 
        std::bind(&DStarLitePlannerNode::plan_and_move, this));
}

void DStarLitePlannerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!last_map_data_ || last_map_data_->info.resolution != msg->info.resolution) 
        planner_ = std::make_unique<planning::DStarLitePlanner>(msg->info.resolution);
    last_map_data_ = msg;
    planner_->setOccupancyGrid(msg);
}

void DStarLitePlannerNode::goal_callback(const oxebots_interfaces::msg::RobotGoal::SharedPtr msg) {
    if (msg->robot_id == (int)robot_id_) { 
        std::lock_guard<std::mutex> lock(data_mutex_); 
        target_goal_msg_ = msg; 
    }
}

void DStarLitePlannerNode::game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto& a : msg->robots.allies) {
        if (a.id == robot_id_) {
            current_pos_world_.x = (float)a.x / 1000.0;
            current_pos_world_.y = (float)a.y / 1000.0;
            current_pos_world_.orientation = (float)a.orientation;
        }
    }
    
    std::vector<movement::Coordinate> allies;
    for (const auto& a : msg->robots.allies) {
        if (a.id != robot_id_) {
            allies.push_back({(float)a.x / 1000.0f, (float)a.y / 1000.0f, (float)a.orientation});
        }
    }
    if(planner_) planner_->setAllyPositions(allies);
}

void DStarLitePlannerNode::follow_path_point_P(const geometry_msgs::msg::PoseStamped& next) {
    auto msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
    oxebots_interfaces::msg::RobotCmdData d;
    d.id = robot_id_;
    
    double dx = next.pose.position.x - current_pos_world_.x;
    double dy = next.pose.position.y - current_pos_world_.y;
    double dist = std::hypot(dx, dy);
    
    if (dist > 0.02) {
        d.x_velocity = (dx/dist) * std::min(1.0, dist * 0.8);
        d.y_velocity = (dy/dist) * std::min(1.0, dist * 0.8);
    }

    if (target_goal_msg_.has_value()) {
        const auto& q = target_goal_msg_.value()->pose.pose.orientation;
        double yaw_goal = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
        double angle_error = normalizeAngle(yaw_goal - current_pos_world_.orientation);
        d.angular_velocity = angle_error * 2.0; 
    }

    msg->robots.push_back(d);
    cmd_vel_pub_->publish(std::move(msg));
}

void DStarLitePlannerNode::plan_and_move() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!planner_ || !target_goal_msg_.has_value() || !last_map_data_) return;
    
    auto origin = last_map_data_->info.origin.position;
    auto target = target_goal_msg_.value()->pose.pose.position;
    
    planning::GridCell s = planner_->worldToGrid(current_pos_world_.x, current_pos_world_.y, origin.x, origin.y);
    planning::GridCell g = planner_->worldToGrid(target.x, target.y, origin.x, origin.y);
    
    if (!planner_->getGoal().has_value() || planner_->getGoal().value() != g) planner_->initialize(s, g);
    
    planner_->setStart(s);
    planner_->checkAndModifyCosts(s);
    
    if (planner_->computePath()) {
        auto path = planner_->reconstructPath(s, origin.x, origin.y);
        path.header.frame_id = last_map_data_->header.frame_id;
        path.header.stamp = this->now();
        path_pub_->publish(path);
        if (path.poses.size() > 1) follow_path_point_P(path.poses[1]);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DStarLitePlannerNode>());
    rclcpp::shutdown();
    return 0;
}