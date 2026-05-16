#include "oxebots_strategy/a_star_planner.hpp"
#include <queue>
#include <vector>
#include <algorithm>
#include <map>

namespace oxebots_strategy
{

AStarPlanner::AStarPlanner(int grid_width, int grid_height)
  : grid_width_(grid_width), grid_height_(grid_height) {}

AStarPlanner::~AStarPlanner() {}

double AStarPlanner::calculateHeuristic(const Node& a, const Node& b)
{
  // Distância Euclidiana
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

bool AStarPlanner::isValid(int y, int x, const std::vector<std::vector<int>>& grid)
{
  return y >= 0 && y < grid_height_ && x >= 0 && x < grid_width_ && grid[y][x] == 0; // 0 significa livre
}

std::vector<Node> AStarPlanner::reconstructPath(const Node& end_node, const std::map<int, Node>& all_nodes)
{
  std::vector<Node> path;
  Node current = end_node;
  while (current.parent_key != -1) {
    path.push_back(current);
    auto it = all_nodes.find(current.parent_key);
    if (it == all_nodes.end()) {
      break; 
    }
    current = it->second;
  }
  path.push_back(current);
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<Node> AStarPlanner::findPath(const Node& start_node, const Node& end_node, const std::vector<std::vector<int>>& grid)
{
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
  std::map<int, Node> all_nodes; // Para armazenar e gerenciar os nós

  int start_key = start_node.y * grid_width_ + start_node.x;
  all_nodes[start_key] = start_node;
  all_nodes[start_key].h_cost = calculateHeuristic(start_node, end_node);

  open_list.push(all_nodes[start_key]);

  std::vector<bool> closed_list(grid_width_ * grid_height_, false);

  // Movimentos possíveis (8 direções)
  int dy[] = {-1, 1, 0, 0, -1, -1, 1, 1};
  int dx[] = {0, 0, -1, 1, -1, 1, -1, 1};

  while (!open_list.empty()) {
    Node current_node = open_list.top();
    open_list.pop();

    int current_key = current_node.y * grid_width_ + current_node.x;

    if (current_node.y == end_node.y && current_node.x == end_node.x) {
      return reconstructPath(current_node, all_nodes);
    }

    if (closed_list[current_key]) {
      continue;
    }
    closed_list[current_key] = true;

    for (int i = 0; i < 8; ++i) {
      int new_y = current_node.y + dy[i];
      int new_x = current_node.x + dx[i];

      if (isValid(new_y, new_x, grid)) {
        int neighbor_key = new_y * grid_width_ + new_x;
        double new_g_cost = current_node.g_cost + ((i < 4) ? 1.0 : 1.414); // Custo maior para diagonais

        if (all_nodes.find(neighbor_key) == all_nodes.end() || new_g_cost < all_nodes[neighbor_key].g_cost) {
          Node neighbor_node(new_y, new_x);
          neighbor_node.parent_key = current_key;
          neighbor_node.g_cost = new_g_cost;
          neighbor_node.h_cost = calculateHeuristic(neighbor_node, end_node);
          
          all_nodes[neighbor_key] = neighbor_node;
          open_list.push(neighbor_node);
        }
      }
    }
  }

  return {}; // Retorna caminho vazio se não encontrar
}

} // namespace oxebots_strategy
