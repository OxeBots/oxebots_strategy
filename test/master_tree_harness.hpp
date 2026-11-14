// Harness da Master Tree (T05): carrega a MainTreeEL REAL e todos os XML de behavior_trees/ (como o
// strategy_node faz), mas com folhas FALSAS e uma blackboard controlada pelo teste (Scenario).
//
// Como as folhas falsas são criadas: o harness registra os nós reais numa fábrica auxiliar só para ler
// os manifestos (ID, tipo, portas) e, na fábrica da árvore, registra para cada folha um falso com o
// MESMO manifesto. Assim o XML é validado contra as portas reais e uma folha nova no node_registry.cpp
// entra no harness sozinha. Ficam REAIS as condições que só leem a blackboard (CheckGameState,
// WeAreFavored, IsDataFresh, HasRole, IsCornerKick, IsThrowIn): são elas que escolhem o ramo.
//
// O observador de ramo repete o StrategyNode::attachBranchObserver() (strategy_node.cpp) de propósito:
// extraí-lo mexeria num arquivo compartilhado. Se aquele critério mudar, mude aqui também.
#pragma once

#include <algorithm>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "master_tree_scenario.hpp"
#include "oxebots_strategy/game_conditions.hpp"
#include "oxebots_strategy/node_registry.hpp"
#include "rclcpp/rclcpp.hpp"

#ifndef MASTER_TREES_DIR
#error "MASTER_TREES_DIR deve apontar para behavior_trees/ (definido no CMakeLists.txt)"
#endif

namespace master_tree_test
{

struct LeafStats
{
  int ticks{0};
  int starts{0};
  int halts{0};
};

// Estado compartilhado entre as folhas falsas e o harness.
struct SharedState
{
  std::map<std::string, LeafStats> stats;
  // O que rodou no tick atual: "ID" e, quando a folha tem a porta reason, "ID:reason".
  std::set<std::string> ran;
  // Último valor CRU de cada porta (literal do XML ou "{chave}") da última execução da folha.
  std::map<std::string, std::map<std::string, std::string>> raw_ports;
  // Status que a folha devolve; o que não estiver aqui usa defaultStatus().
  std::map<std::string, BT::NodeStatus> status;
};

// Folhas que ficam em RUNNING até serem interrompidas (ações assíncronas do robô).
inline const std::set<std::string> & runningLeafIds()
{
  static const std::set<std::string> ids{
    "GoToPoint", "HoldPosition", "Goalkeeper", "KickOnce", "KickBall", "AlignToBall",
    "GoalLineDefend", "LateralClear", "PositionBisector", "SmotherSave", "GoToClamped",
    "PrecisionKick"};
  return ids;
}

inline BT::NodeStatus defaultStatus(const std::string & id)
{
  static const std::set<std::string> failing{
    "IsBallClose", "IsBallInArea", "IsBallClosest", "IsBallInOurHalf", "IsBallInOpponentField",
    "IsOpponentControllingBall"};
  if (runningLeafIds().count(id) > 0) {
    return BT::NodeStatus::RUNNING;
  }
  if (failing.count(id) > 0) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

inline BT::NodeStatus recordLeafTick(
  SharedState & shared, const std::string & id, const BT::NodeConfig & config)
{
  ++shared.stats[id].ticks;
  shared.ran.insert(id);
  for (const auto & [port, raw] : config.input_ports) {
    shared.raw_ports[id][port] = raw;
  }
  const auto reason = config.input_ports.find("reason");
  if (reason != config.input_ports.end()) {
    shared.ran.insert(id + ":" + reason->second);
  }
  const auto configured = shared.status.find(id);
  return configured != shared.status.end() ? configured->second : defaultStatus(id);
}

class FakeAction : public BT::StatefulActionNode
{
public:
  FakeAction(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<SharedState> shared, std::string id)
  : BT::StatefulActionNode(name, config), shared_(std::move(shared)), id_(std::move(id)) {}

  BT::NodeStatus onStart() override
  {
    ++shared_->stats[id_].starts;
    return recordLeafTick(*shared_, id_, config());
  }

  BT::NodeStatus onRunning() override {return recordLeafTick(*shared_, id_, config());}

  void onHalted() override {++shared_->stats[id_].halts;}

private:
  std::shared_ptr<SharedState> shared_;
  std::string id_;
};

class FakeCondition : public BT::ConditionNode
{
public:
  FakeCondition(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<SharedState> shared, std::string id)
  : BT::ConditionNode(name, config), shared_(std::move(shared)), id_(std::move(id)) {}

  BT::NodeStatus tick() override
  {
    ++shared_->stats[id_].starts;
    return recordLeafTick(*shared_, id_, config());
  }

private:
  std::shared_ptr<SharedState> shared_;
  std::string id_;
};

class MasterTreeHarness
{
public:
  explicit MasterTreeHarness(const std::string & main_xml = "master_strategy_el.xml")
  : shared_(std::make_shared<SharedState>())
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    ros_node_ = std::make_shared<rclcpp::Node>("master_tree_harness_registry");

    BT::BehaviorTreeFactory real;
    oxebots_strategy::registerStrategyNodes(real, ros_node_);
    const std::set<std::string> builtin = real.builtinNodes();

    registerRealConditions();
    for (const auto & entry : real.manifests()) {
      // Cópia local: binding estruturado não pode ser capturado por lambda em C++17.
      const std::string id = entry.first;
      const BT::TreeNodeManifest & manifest = entry.second;
      if (builtin.count(id) > 0 || isRealCondition(id)) {
        continue;
      }
      auto shared = shared_;
      const bool is_condition = manifest.type == BT::NodeType::CONDITION;
      factory_.registerBuilder(
        manifest,
        [shared, id, is_condition](const std::string & name, const BT::NodeConfig & config)
        -> std::unique_ptr<BT::TreeNode> {
          if (is_condition) {
            return std::make_unique<FakeCondition>(name, config, shared, id);
          }
          return std::make_unique<FakeAction>(name, config, shared, id);
        });
    }

    // Mesmo carregamento do strategy_node: todos os .xml do diretório (sem legacy/) viram subárvores.
    const std::filesystem::path dir{MASTER_TREES_DIR};
    const std::filesystem::path main_path = dir / main_xml;
    std::vector<std::filesystem::path> others;
    for (const auto & entry : std::filesystem::directory_iterator(dir)) {
      if (entry.path().extension() == ".xml" && entry.path() != main_path &&
        entry.path().filename() != "tree_nodes_model.xml")
      {
        others.push_back(entry.path());
      }
    }
    std::sort(others.begin(), others.end());
    for (const auto & path : others) {
      factory_.registerBehaviorTreeFromFile(path);
    }

    blackboard_ = BT::Blackboard::create();
    scenario.applyTo(*blackboard_);
    tree_ = factory_.createTreeFromFile(main_path.string(), blackboard_);
    attachBranchObserver();
  }

  MasterTreeHarness(const MasterTreeHarness &) = delete;
  MasterTreeHarness & operator=(const MasterTreeHarness &) = delete;

  // Estado de jogo, papel, idades e mundo do próximo tick (reescrito na blackboard a cada tick,
  // como o refresh() faz).
  Scenario scenario;

  // Chave que o refresh() deixaria sem valor: removida da blackboard depois de aplicar o cenário.
  void omit(const std::string & key) {omitted_.insert(key);}

  void setStatus(const std::string & id, BT::NodeStatus status) {shared_->status[id] = status;}

  BT::NodeStatus tick()
  {
    scenario.applyTo(*blackboard_);
    for (const auto & key : omitted_) {
      blackboard_->unset(key);
    }
    shared_->ran.clear();
    selected_.clear();

    const BT::NodeStatus status = tree_.tickOnce();

    branch_.clear();
    for (BT::TreeNode * child : branches_) {
      if (child != nullptr && child->status() == BT::NodeStatus::RUNNING) {
        branch_ = child->name();
        break;
      }
    }
    if (branch_.empty()) {
      branch_ = selected_;
    }
    return status;
  }

  void tick(int count)
  {
    for (int i = 0; i < count; ++i) {
      tick();
    }
  }

  void haltTree() {tree_.haltTree();}

  // Ramo da raiz que decidiu o último tick (nome do filho da raiz).
  const std::string & branch() const {return branch_;}

  const std::vector<std::string> & rootBranchNames() const {return branch_names_;}

  // A folha (ID) rodou no último tick? Para o Halt também vale "Halt:<reason>".
  bool ran(const std::string & id) const {return shared_->ran.count(id) > 0;}

  const std::set<std::string> & ranSet() const {return shared_->ran;}

  LeafStats stats(const std::string & id) const
  {
    const auto it = shared_->stats.find(id);
    return it != shared_->stats.end() ? it->second : LeafStats{};
  }

  // Valor cru da porta na última execução da folha ("" se nunca rodou ou a porta não foi dada).
  std::string rawPort(const std::string & id, const std::string & port) const
  {
    const auto leaf = shared_->raw_ports.find(id);
    if (leaf == shared_->raw_ports.end()) {
      return "";
    }
    const auto value = leaf->second.find(port);
    return value != leaf->second.end() ? value->second : "";
  }

  // Folhas assíncronas que rodaram no último tick e continuam RUNNING.
  std::set<std::string> runningLeaves() const
  {
    std::set<std::string> out;
    for (const auto & id : shared_->ran) {
      if (runningLeafIds().count(id) > 0 &&
        (shared_->status.count(id) == 0 || shared_->status.at(id) == BT::NodeStatus::RUNNING))
      {
        out.insert(id);
      }
    }
    return out;
  }

  // Quantidade total de halt() recebidos pelas folhas assíncronas.
  int totalHalts() const
  {
    int total = 0;
    for (const auto & entry : shared_->stats) {
      total += entry.second.halts;
    }
    return total;
  }

private:
  static bool isRealCondition(const std::string & id)
  {
    static const std::set<std::string> ids{
      "CheckGameState", "WeAreFavored", "IsDataFresh", "HasRole", "IsCornerKick", "IsThrowIn"};
    return ids.count(id) > 0;
  }

  void registerRealConditions()
  {
    factory_.registerNodeType<oxebots_strategy::CheckGameStateCondition>("CheckGameState");
    factory_.registerNodeType<oxebots_strategy::WeAreFavoredCondition>("WeAreFavored");
    factory_.registerNodeType<oxebots_strategy::IsDataFreshCondition>("IsDataFresh");
    factory_.registerNodeType<oxebots_strategy::HasRoleCondition>("HasRole");
    factory_.registerNodeType<oxebots_strategy::IsCornerKickCondition>("IsCornerKick");
    factory_.registerNodeType<oxebots_strategy::IsThrowInCondition>("IsThrowIn");
  }

  void attachBranchObserver()
  {
    BT::TreeNode * root = tree_.rootNode();
    while (auto * decorator = dynamic_cast<BT::DecoratorNode *>(root)) {
      root = decorator->child();
    }
    auto * control = dynamic_cast<BT::ControlNode *>(root);
    if (control == nullptr) {
      throw std::runtime_error("A raiz da árvore mestre não é um nó de controle");
    }
    for (BT::TreeNode * child : control->children()) {
      if (child == nullptr) {
        continue;
      }
      branches_.push_back(child);
      branch_names_.push_back(child->name());
      subscriptions_.push_back(
        child->subscribeToStatusChange(
          [this](BT::TimePoint, const BT::TreeNode & node, BT::NodeStatus, BT::NodeStatus status)
          {
            if (status == BT::NodeStatus::SUCCESS) {
              selected_ = node.name();
            }
          }));
    }
  }

  std::shared_ptr<SharedState> shared_;
  rclcpp::Node::SharedPtr ros_node_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
  std::set<std::string> omitted_;

  std::vector<BT::TreeNode *> branches_;
  std::vector<std::string> branch_names_;
  std::vector<BT::TreeNode::StatusChangeSubscriber> subscriptions_;
  std::string selected_;
  std::string branch_;
};

}  // namespace master_tree_test
