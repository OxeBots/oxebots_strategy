// Despeja o TreeNodesModel (portas e padrões de todas as folhas) para o validate_bt.py e Groot2.
//
// Usa o registro único de oxebots_strategy::registerStrategyNodes (src/node_registry.cpp).
// Sem argumentos: imprime todos os nós (incluindo os nativos da biblioteca) + comentário
//   <!-- builtin_nodes: ... --> para o scripts/validate_bt.py.
// Com --groot: imprime apenas as folhas customizadas do projeto em formato TreeNodesModel XML
//   para importação direta no Groot2.
#include <csignal>
#include <iostream>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "oxebots_strategy/node_registry.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
#ifdef SIGPIPE
  std::signal(SIGPIPE, SIG_IGN);
#endif
  bool groot_only = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--groot") {
      groot_only = true;
    }
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dump_bt_model");

  BT::BehaviorTreeFactory factory;
  oxebots_strategy::registerStrategyNodes(factory, node);

  if (groot_only) {
    std::cout << oxebots_strategy::generateTreeNodesModelXML(factory, false) << "\n";
  } else {
    std::cout << oxebots_strategy::generateTreeNodesModelXML(factory, true) << "\n";
    std::cout << "<!-- builtin_nodes:";
    for (const auto & id : factory.builtinNodes()) {
      std::cout << ' ' << id;
    }
    std::cout << " -->\n";
  }

  rclcpp::shutdown();
  return 0;
}
