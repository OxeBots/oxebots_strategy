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

#include "oxebots_strategy/d_star_planner.h"

#include <iomanip>

namespace planning
{

DStarPlanner::DStarPlanner(double resolution, const PlannerConfig & config, rclcpp::Logger logger)
: resolution_(resolution), config_(config), logger_(logger)
{
    RCLCPP_INFO(logger_, "D* Planner initialized with resolution %.3f m", resolution_);
}

bool DStarPlanner::OpenListEntry::operator>(const OpenListEntry & other) const
{
    // Tie-breaking mechanism for priority queue
    if (std::abs(sort_key - other.sort_key) > 1e-7)
        return sort_key > other.sort_key;
    return raw_k > other.raw_k;
}

void DStarPlanner::setOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr & grid)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!grid)
        return;

    // Check if dimensions changed. We only re-allocate if the map size changes.
    // This is crucial for performance to avoid heap fragmentation and allocation overhead.
    bool dim_changed =
      (static_cast<int>(grid->info.width) != grid_width_ || static_cast<int>(grid->info.height) != grid_height_);

    current_grid_ = grid;
    grid_width_ = grid->info.width;
    grid_height_ = grid->info.height;

    // Re-allocate graph ONLY if dimensions change (Avoid heap churn)
    if (dim_changed)
    {
        grid_nodes_.clear();
        grid_nodes_.reserve(grid_width_ * grid_height_);
        dynamic_obstacle_map_.assign(grid_width_ * grid_height_, false);
        for (int y = 0; y < grid_height_; ++y)
        {
            for (int x = 0; x < grid_width_; ++x)
            {
                DStarNode node;
                node.cell = {x, y};
                grid_nodes_.push_back(node);
            }
        }
        RCLCPP_INFO(logger_, "Map resized to %dx%d (%zu nodes allocated)", grid_width_, grid_height_,
                    grid_nodes_.size());
    }
}

void DStarPlanner::setAllyPositions(const std::vector<std::pair<float, float>> & allies)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!current_grid_)
        return;

    // Identificar células que eram obstáculos para limpá-las no D*
    std::vector<GridCell> old_obstacles = dynamic_obstacles_;
    for (const auto & obs : old_obstacles) {
        int idx = obs.y * grid_width_ + obs.x;
        if (idx >= 0 && idx < (int)dynamic_obstacle_map_.size()) dynamic_obstacle_map_[idx] = false;
    }
    dynamic_obstacles_.clear();

    double ox = current_grid_->info.origin.position.x;
    double oy = current_grid_->info.origin.position.y;
    int radius_cells = std::ceil(config_.robot_safety_radius / resolution_);

    // Converter posições dos aliados para células da grade e marcar como obstáculos
    for (const auto & ally : allies)
    {
        GridCell center = worldToGrid(ally.first, ally.second, ox, oy);

        for (int dx = -radius_cells; dx <= radius_cells; ++dx)
        {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy)
            {
                GridCell cell = {center.x + dx, center.y + dy};
                // Apenas adiciona se estiver dentro dos limites do mapa
                if (cell.x >= 0 && cell.x < grid_width_ && cell.y >= 0 && cell.y < grid_height_) {
                    dynamic_obstacles_.push_back(cell);
                    dynamic_obstacle_map_[cell.y * grid_width_ + cell.x] = true;
                }
            }
        }
    }

    // Notificar o D* sobre as mudanças de custo (tanto as antigas quanto as novas)
    // Isso garante que o caminho seja recalculado se um robô aliado se mover.
    for (const auto & obs : old_obstacles)
    {
        if (auto node = getNode(obs.x, obs.y))
            modifyCost(node);
    }
    for (const auto & obs : dynamic_obstacles_)
    {
        if (auto node = getNode(obs.x, obs.y))
            modifyCost(node);
    }
}

DStarNode * DStarPlanner::getNode(int x, int y)
{
    // Bounds check
    if (x < 0 || y < 0 || x >= grid_width_ || y >= grid_height_)
        return nullptr;
    // Map 2D coordinate to 1D vector index
    return &grid_nodes_[y * grid_width_ + x];
}

double DStarPlanner::calculateHeuristic(const GridCell & a, const GridCell & b) const
{
    double dx = std::abs(a.x - b.x);
    double dy = std::abs(a.y - b.y);

    switch (config_.heuristic_type)
    {
        case HeuristicType::MANHATTAN:
            return dx + dy;
        case HeuristicType::DIAGONAL:
            // Standard Octile distance
            return (dx + dy) + (M_SQRT2 - 2) * std::min(dx, dy);
        default:
            // Euclidean distance with slight tie-breaker to prefer straight paths
            return std::hypot(dx, dy) * 1.2;
    }
}

double DStarPlanner::getSiteCost(const GridCell & u) const
{
    // CRÍTICO: Sempre tratar a vizinhança imediata do Início e do Alvo como navegável.
    // Isso evita que o robô trave se o alvo (bola) for marcado como obstáculo 
    // ou se o robô estiver ligeiramente dentro de uma zona de inflação.
    if (start_cell_.has_value()) {
        if (std::abs(u.x - start_cell_->x) <= 4 && std::abs(u.y - start_cell_->y) <= 4) return 1.0;
    }
    if (goal_cell_.has_value()) {
        if (std::abs(u.x - goal_cell_->x) <= 8 && std::abs(u.y - goal_cell_->y) <= 8) return 1.0;
    }

    if (!current_grid_)
        return std::numeric_limits<double>::infinity();

    // Bounds check
    if (u.x < 0 || u.y < 0 || u.x >= grid_width_ || u.y >= grid_height_)
        return std::numeric_limits<double>::infinity();

    // Check Static Map
    int idx = u.y * grid_width_ + u.x;
    if (current_grid_->data[idx] >= config_.occupancy_threshold)
        return std::numeric_limits<double>::infinity();

    // Check Dynamic Obstacles (O(1) lookup)
    if (dynamic_obstacle_map_[idx])
        return std::numeric_limits<double>::infinity();

    return 1.0;  // Traversable
}

double DStarPlanner::getTraversalCost(DStarNode * n1, DStarNode * n2) const
{
    double c1 = getSiteCost(n1->cell);
    double c2 = getSiteCost(n2->cell);

    if (std::isinf(c1) || std::isinf(c2))
        return std::numeric_limits<double>::infinity();

    // Calculate distance multiplier (sqrt(2) for diagonal, 1.0 for cardinal)
    double dist = (n1->cell.x != n2->cell.x && n1->cell.y != n2->cell.y) ? M_SQRT2 : 1.0;
    return ((c1 + c2) / 2.0) * dist;
}

std::vector<DStarNode *> DStarPlanner::getNeighbors(DStarNode * u)
{
    std::vector<DStarNode *> neighbors;
    neighbors.reserve(8);

    // 8-connected grid
    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            if (dx == 0 && dy == 0)
                continue;

            if (auto node = getNode(u->cell.x + dx, u->cell.y + dy))
                neighbors.push_back(node);
        }
    }
    return neighbors;
}

void DStarPlanner::insert(DStarNode * node, double h_new)
{
    // Update k value based on node status (D* logic)
    switch (node->tag)
    {
        case Tag::NEW:
            node->k = h_new;
            break;
        case Tag::OPEN:
            node->k = std::min(node->k, h_new);
            break;
        case Tag::CLOSED:
            node->k = std::min(node->h, h_new);
            break;
    }

    node->h = h_new;
    node->tag = Tag::OPEN;

    // Calculate Priority Key: f = k + h_heuristic
    double h_val = start_cell_ ? calculateHeuristic(node->cell, *start_cell_) : 0.0;
    open_list_.push({node->k + h_val, node->k, node});
}

double DStarPlanner::processState()
{
    if (open_list_.empty())
        return -1.0;

    // Lazy Removal: Pop items from PQ until we find a valid one or PQ is empty
    OpenListEntry top;
    do
    {
        if (open_list_.empty())
            return -1.0;
        top = open_list_.top();
        open_list_.pop();
    } while (top.node->tag == Tag::CLOSED || (top.node->tag == Tag::OPEN && std::abs(top.node->k - top.raw_k) > 1e-5));

    DStarNode * x = top.node;
    double k_old = top.raw_k;
    x->tag = Tag::CLOSED;
    metrics_.total_expansions++;

    // RAISE State: The node became more expensive (obstacle appeared)
    if (x->h > k_old + 1e-5)
    {
        for (auto y : getNeighbors(x))
        {
            double cost = getTraversalCost(x, y);
            if (y->tag != Tag::NEW && y->h <= k_old - 1e-5 && x->h > y->h + cost + 1e-5)
            {
                x->parent = y;
                x->h = y->h + cost;
            }
        }
    }

    // NORMAL State: Standard expansion
    if (std::abs(x->h - k_old) <= 1e-5)
    {
        for (auto y : getNeighbors(x))
        {
            double cost = getTraversalCost(x, y);
            bool is_new = (y->tag == Tag::NEW);
            bool parent_cost_mismatch = (y->parent == x && std::abs(y->h - (x->h + cost)) > 1e-5);
            bool shorter_path_found = (y->parent != x && y->h > x->h + cost + 1e-5);

            if (is_new || parent_cost_mismatch || shorter_path_found)
            {
                y->parent = x;
                insert(y, x->h + cost);
            }
        }
    }
    // LOWER / RAISE Propagation
    else
    {
        for (auto y : getNeighbors(x))
        {
            double cost = getTraversalCost(x, y);
            if (y->tag == Tag::NEW || (y->parent == x && std::abs(y->h - (x->h + cost)) > 1e-5))
            {
                y->parent = x;
                insert(y, x->h + cost);
            }
            else if (y->parent != x && y->h > x->h + cost + 1e-5 && x->tag == Tag::CLOSED)
                insert(x, x->h);
            else if (y->parent != x && x->h > y->h + getTraversalCost(y, x) + 1e-5 && y->tag == Tag::CLOSED &&
                     y->h > k_old + 1e-5)
                insert(y, y->h);
        }
    }
    return open_list_.empty() ? -1.0 : open_list_.top().raw_k;
}

void DStarPlanner::modifyCost(DStarNode * node)
{
    // If a closed node is modified, we insert it back into OPEN with its current cost. D* will then
    // pick it up and process the change (RAISE or LOWER).
    if (node->tag == Tag::CLOSED)
        insert(node, node->h);
}

void DStarPlanner::initialize(const GridCell & start, const GridCell & goal)
{
    auto t1 = std::chrono::steady_clock::now();

    // Clear Open List
    while (!open_list_.empty())
        open_list_.pop();

    // Reset Nodes efficiently (avoid reallocating)
    for (auto & node : grid_nodes_)
        node.reset();

    start_cell_ = start;
    goal_cell_ = goal;

    // D* searches backwards from Goal to Start
    if (auto g_node = getNode(goal.x, goal.y))
        insert(g_node, 0.0);

    metrics_.planning_time =
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t1);

    RCLCPP_DEBUG(logger_, "Initialized D* in %ld us. Start(%d,%d) Goal(%d,%d)", metrics_.planning_time.count(), start.x,
                 start.y, goal.x, goal.y);
}

PlannerStatus DStarPlanner::plan()
{
    auto t1 = std::chrono::steady_clock::now();

    if (!start_cell_ || !goal_cell_)
        return PlannerStatus::INVALID_START_GOAL;

    auto s_node = getNode(start_cell_->x, start_cell_->y);

    if (!s_node)
        return PlannerStatus::FAILURE;

    int steps = 0;
    // Process open list until start node is closed (meaning we found a path)
    // or we run out of nodes/expansions.
    while (s_node->tag != Tag::CLOSED && !open_list_.empty() && steps++ < config_.max_expansions)
    {
        processState();
        if (steps % 2000 == 0)
        {
            RCLCPP_DEBUG(logger_, "Planning... %d expansions, cur cost: %.2f", steps, s_node->h);
        }
    }
    auto t2 = std::chrono::steady_clock::now();
    metrics_.planning_time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

    if (s_node->tag == Tag::CLOSED)
    {
        metrics_.path_cost = s_node->h;
        RCLCPP_INFO(logger_, "Path found: %d expansions, cost %.2f after %ld us", steps, s_node->h,
                    metrics_.planning_time.count());
        return PlannerStatus::SUCCESS;
    }

    RCLCPP_WARN(logger_, "Plan failed after %d expansions after %ld us", steps, metrics_.planning_time.count());
    return PlannerStatus::FAILURE;
}

void DStarPlanner::updateMapAndReplan(const GridCell & current_pos)
{
    auto t1 = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!start_cell_)
        return;

    std::vector<DStarNode *> changed;
    int range = static_cast<int>(config_.sensor_range);

    // Identify Changed Nodes
    // We scan a window around the robot. In a real system, we'd compare the old map vs new map to
    // only flag actual changes. Here we assume potential changes.
    for (int dx = -range; dx <= range; ++dx)
    {
        for (int dy = -range; dy <= range; ++dy)
        {
            auto u = getNode(current_pos.x + dx, current_pos.y + dy);
            if (!u)
                continue;

            changed.push_back(u);
        }
    }

    // Flag changed nodes for processing
    for (auto n : changed)
        modifyCost(n);

    // Repair Path
    // We only process until the minimum key in OPEN is greater than the start node's cost.
    // This implies that the current path is optimal/valid given the changes.
    auto s_node = getNode(start_cell_->x, start_cell_->y);
    int steps = 0;
    while (!open_list_.empty() && steps++ < config_.max_replan_steps)
    {
        if (open_list_.top().raw_k >= s_node->h)
            break;
        processState();
    }

    auto t2 = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

    if (steps > 0)
        RCLCPP_DEBUG(logger_, "Replan: %ld us, %d steps, %zu changed nodes", duration, steps, changed.size());
}

nav_msgs::msg::Path DStarPlanner::reconstructPath(const GridCell & start, double ox, double oy)
{
    nav_msgs::msg::Path path;
    auto current = getNode(start.x, start.y);

    if (!current || std::isinf(current->h))
        return path;

    // Trace back parent pointers from Start -> Goal
    int steps = 0;
    while (current && steps++ < 1000)
    {
        path.poses.push_back(gridToWorld(current->cell, ox, oy));

        if (goal_cell_ && current->cell == *goal_cell_)
            break;

        current = current->parent;
    }
    metrics_.path_length = path.poses.size();

    if (path.poses.empty())
        RCLCPP_WARN(logger_, "Reconstructed path is empty (start: %d,%d)", start.x, start.y);
    return path;
}

GridCell DStarPlanner::worldToGrid(double wx, double wy, double ox, double oy) const
{
    return {static_cast<int>(std::floor((wx - ox) / resolution_)),
            static_cast<int>(std::floor((wy - oy) / resolution_))};
}

geometry_msgs::msg::PoseStamped DStarPlanner::gridToWorld(const GridCell & gc, double ox, double oy) const
{
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = ox + (gc.x + 0.5) * resolution_;
    ps.pose.position.y = oy + (gc.y + 0.5) * resolution_;
    return ps;
}

DStarPlannerNode::DStarPlannerNode() : Node("d_star_planner_node")
{
    // Helper to declare parameters safely (checks existence first)
    auto safe_param = [&](const std::string & name, auto def) {
        if (!this->has_parameter(name))
            this->declare_parameter(name, def);
        return this->get_parameter(name);
    };

    robot_id_ = safe_param("robot_id", 0).as_int();
    double hz = safe_param("planning_rate_hz", 10.0).as_double();

    planning::PlannerConfig config;
    config.robot_safety_radius = safe_param("robot_safety_radius", 0.20).as_double();
    config.occupancy_threshold = safe_param("occupancy_threshold", 80).as_int();
    config.max_expansions = safe_param("max_expansions", 100000).as_int();

    planner_ = std::make_unique<planning::DStarPlanner>(0.05, config, this->get_logger());

    std::string path_topic = "/robot_" + std::to_string(robot_id_) + "/path";
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, 10);
    
    // Tópico para visualização no RViz (acessível para todos os robôs)
    rviz_path_pub_ = create_publisher<nav_msgs::msg::Path>("/robot_" + std::to_string(robot_id_) + "/path_viz", 10);

    // Subscribers with Transient Local QoS for map/goals (latched topics)
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", qos, std::bind(&DStarPlannerNode::map_callback, this, std::placeholders::_1));

    game_data_sub_ = create_subscription<oxebots_interfaces::msg::GameData>(
      "game_data", 10, std::bind(&DStarPlannerNode::game_data_callback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<oxebots_interfaces::msg::RobotGoal>(
      "/robot_goal", qos, std::bind(&DStarPlannerNode::goal_callback, this, std::placeholders::_1));

    geometry_sub_ = create_subscription<oxebots_interfaces::msg::SSLGeometryData>(
    "/field_geometry", qos, std::bind(&DStarPlannerNode::geometry_callback, this, std::placeholders::_1));

    planning_timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
                                        std::bind(&DStarPlannerNode::plan_and_publish, this));
}

void DStarPlannerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(node_mutex_);
    
    // 1. Criamos uma cópia mutável do mapa para podermos "desenhar" nele 
    // sem afetar os mapas dos robôs atacantes
    auto custom_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg);

    // 2. Se for o zagueiro (robô 2) e já tivermos a geometria, criamos a Parede de Vidro!
    if (robot_id_ == 2 && last_geometry_) {
        double res = custom_map->info.resolution;
        double origin_x = custom_map->info.origin.position.x;
        double origin_y = custom_map->info.origin.position.y;
        int width = custom_map->info.width;
        int height = custom_map->info.height;

        // A geometria da SSL vem em milímetros, mas o D* e o mapa usam METROS
        double field_length = last_geometry_->field.field_length / 1000.0;
        double penalty_depth = last_geometry_->field.penalty_area_depth / 1000.0;
        double penalty_width = last_geometry_->field.penalty_area_width / 1000.0;

        // Calcula a linha exata da área
        double penalty_x = (field_length / 2.0) - penalty_depth;
        double penalty_y_min = -penalty_width / 2.0;
        double penalty_y_max = penalty_width / 2.0;

        // Adiciona a margem de segurança do robô (ex: 9cm = 0.09m) para as rodas não pisarem na linha
        double margin = 0.09;
        double safe_penalty_x = penalty_x - margin;
        double safe_y_min = penalty_y_min - margin;
        double safe_y_max = penalty_y_max + margin;

        // Varre absolutamente TODAS as células do mapa
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // Converte de "índice da grade" para "metros reais no campo"
                double world_x = origin_x + (x * res);
                double world_y = origin_y + (y * res);

                // Bloqueia as áreas de pênalti de AMBOS os lados (o zagueiro não deve entrar em nenhuma área)
                bool in_positive_area = (world_x > safe_penalty_x && world_y > safe_y_min && world_y < safe_y_max);
                bool in_negative_area = (world_x < -safe_penalty_x && world_y > safe_y_min && world_y < safe_y_max);

                if (in_positive_area || in_negative_area) {
                    // O peso 100 transforma o espaço vazio em um bloco de concreto impenetrável para o D*
                    custom_map->data[y * width + x] = 100; 
                }
            }
        }
    }

    // 3. Salva e envia o mapa "grafitado" com as barreiras para o cérebro do D* Planner
    last_map_data_ = custom_map;
    if (planner_) {
        planner_->setOccupancyGrid(last_map_data_);
    }
}

void DStarPlannerNode::geometry_callback(const oxebots_interfaces::msg::SSLGeometryData::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(node_mutex_);
    last_geometry_ = msg;
}

void DStarPlannerNode::goal_callback(const oxebots_interfaces::msg::RobotGoal::SharedPtr msg)
{
    if (msg->robot_id == robot_id_)
    {
        std::lock_guard<std::mutex> lock(node_mutex_);
        target_goal_msg_ = msg;
        consecutive_failures_ = 0;
    }
}

void DStarPlannerNode::game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(node_mutex_);
    std::vector<std::pair<float, float>> allies;

    for (const auto & r : msg->robots.allies)
    {
        if (static_cast<int>(r.id) == robot_id_)
        {
            current_x_ = r.x / 1000.0f;
            current_y_ = r.y / 1000.0f;
        }
        else
            allies.push_back({r.x / 1000.0f, r.y / 1000.0f});
    }

    // Adiciona a BOLA como um obstáculo dinâmico para evitar colidir com ela "sem querer"
    allies.push_back({msg->ball.x / 1000.0f, msg->ball.y / 1000.0f});

    if (planner_)
        planner_->setAllyPositions(allies);
}

void DStarPlannerNode::plan_and_publish()
{
    std::lock_guard<std::mutex> lock(node_mutex_);
    if (!planner_ || !target_goal_msg_ || !last_map_data_) {
        if (!target_goal_msg_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "D* aguardando objetivo (/robot_goal)...");
        }
        if (!last_map_data_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "D* aguardando mapa (map)...");
        }
        return;
    }

    auto origin = last_map_data_->info.origin.position;
    auto target = target_goal_msg_.value()->pose.pose.position;

    if (robot_id_ == 2 && last_geometry_) {
        // A geometria da SSL vem em milímetros, mas o D* usa METROS.
        // Dividimos por 1000.0 para alinhar a matemática.
        double field_length = last_geometry_->field.field_length / 1000.0;
        double penalty_depth = last_geometry_->field.penalty_area_depth / 1000.0;
        double penalty_width = last_geometry_->field.penalty_area_width / 1000.0;

        double penalty_x = (field_length / 2.0) - penalty_depth;
        double penalty_y_min = -penalty_width / 2.0;
        double penalty_y_max = penalty_width / 2.0;

        // Margem de segurança para o raio do robô (ex: 9cm = 0.09m)
        // Evita que as rodas belisquem a linha da área
        double margin = 0.09;

        if (target.x > (penalty_x - margin) && (target.y > penalty_y_min && target.y < penalty_y_max)) {
            target.x = penalty_x - margin; // Trava o alvo exatamente na linha da área!
        }
    }

    // Convert world coordinates to grid coordinates
    auto s = planner_->worldToGrid(current_x_, current_y_, origin.x, origin.y);
    auto g = planner_->worldToGrid(target.x, target.y, origin.x, origin.y);

    auto cur_goal = planner_->getGoal();
    if (!cur_goal || *cur_goal != g)
    {
        // New goal received: full re-initialization
        planner_->initialize(s, g);
        if (planner_->plan() != planning::PlannerStatus::SUCCESS)
        {
            // Publish empty path on failure to stop the robot
            nav_msgs::msg::Path empty_path;
            empty_path.header.stamp = now();
            empty_path.header.frame_id = "map";
            path_pub_->publish(empty_path);
            return;
        }
    }
    else
    {
        // Check if we are already close enough to the goal (within 5cm)
        double dx = current_x_ - target.x;
        double dy = current_y_ - target.y;
        if (std::sqrt(dx*dx + dy*dy) < 0.05) {
            nav_msgs::msg::Path empty_path;
            empty_path.header.stamp = now();
            empty_path.header.frame_id = "map";
            path_pub_->publish(empty_path);
            target_goal_msg_ = std::nullopt; // Clear goal after reaching it
            return;
        }
        // Same goal: incremental replanning based on map updates
        planner_->updateMapAndReplan(s);
    }

    auto path = planner_->reconstructPath(s, origin.x, origin.y);
    path.header = last_map_data_->header;
    path_pub_->publish(path);
    if (rviz_path_pub_) {
        rviz_path_pub_->publish(path);
    }
}

}  // namespace planning

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<planning::DStarPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
