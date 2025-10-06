#pragma once

#include <vector>
#include <cmath>
#include <memory>
#include <iostream>
#include <map>

namespace oxebots_strategy
{

// Estrutura para representar um ponto/nó no grid do A*
struct Node
{
  int y, x;
  double g_cost = 0.0; // Custo do início até este nó
  double h_cost = 0.0; // Heurística (custo estimado até o alvo)
  int parent_key = -1;

  Node() : y(0), x(0) {} // Construtor padrão
  Node(int r, int c) : y(r), x(c) {}

  double getFCost() const { return g_cost + h_cost; }

  // Sobrecarga para comparação em filas de prioridade
  bool operator>(const Node& other) const {
    return this->getFCost() > other.getFCost();
  }
};

// Classe principal do planejador A*
class AStarPlanner
{
public:
  AStarPlanner(int grid_width, int grid_height);
  ~AStarPlanner();

  // A função principal que encontrará o caminho
  std::vector<Node> findPath(const Node& start_node, const Node& end_node, const std::vector<std::vector<int>>& grid);

private:
  // Funções auxiliares
  double calculateHeuristic(const Node& a, const Node& b);
  bool isValid(int y, int x, const std::vector<std::vector<int>>& grid);
  std::vector<Node> reconstructPath(const Node& end_node, const std::map<int, Node>& all_nodes);

  int grid_width_;
  int grid_height_;
};

} // namespace oxebots_strategy
