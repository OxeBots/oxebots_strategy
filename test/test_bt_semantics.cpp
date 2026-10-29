// Harness de semântica da BehaviorTree.CPP (backlog T01).
//
// A segurança do projeto depende de comportamentos da biblioteca que foram conferidos nas fontes
// da série 4.6, enquanto o container usa a 4.10. Cada teste abaixo reproduz uma afirmação do
// manual (Cap. 4, 5 e 23) com folhas falsas que contam ticks e halts, e roda na versão instalada.
// Um teste que falha NÃO deve ser "ajustado para passar": ele é um achado. Quando a expectativa
// vinha do manual e o comportamento real é outro, o teste passa a documentar o comportamento
// real (com um comentário "Achado na ...") e o manual deve ser corrigido.
//
// Não usa ROS: só behaviortree_cpp e gtest.
#include <gtest/gtest.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace
{

using BT::NodeStatus;

// Contadores por "probe" (nome passado na porta de cada folha falsa).
struct Probe
{
  int ticks = 0;    // chamadas de tick/onStart/onRunning
  int starts = 0;   // chamadas de onStart
  int running = 0;  // chamadas de onRunning
  int halts = 0;    // chamadas de onHalted
};
std::map<std::string, Probe> g_probes;

NodeStatus charToStatus(char c)
{
  switch (c) {
    case 'S': return NodeStatus::SUCCESS;
    case 'F': return NodeStatus::FAILURE;
    default: return NodeStatus::RUNNING;
  }
}

// Folha assíncrona: devolve, a cada chamada, o próximo caractere do roteiro
// (R = RUNNING, S = SUCCESS, F = FAILURE). Repete o último ao acabar o roteiro. O roteiro
// recomeça em cada onStart.
class Scripted : public BT::StatefulActionNode
{
public:
  Scripted(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("script"), BT::InputPort<std::string>("probe")};
  }

  NodeStatus onStart() override
  {
    step_ = 0;
    probe().starts++;
    return next();
  }
  NodeStatus onRunning() override
  {
    probe().running++;
    return next();
  }
  void onHalted() override {probe().halts++;}

private:
  Probe & probe()
  {
    std::string name;
    getInput("probe", name);
    return g_probes[name];
  }
  NodeStatus next()
  {
    std::string script;
    getInput("script", script);
    probe().ticks++;
    const size_t idx = std::min<size_t>(step_, script.size() - 1);
    step_++;
    return charToStatus(script[idx]);
  }
  size_t step_ = 0;
};

// Folha síncrona com roteiro (S ou F). Aborta se for chamada em excesso dentro de um mesmo
// teste, para que um laço infinito da biblioteca vire falha em vez de travar o gtest.
class ScriptedSync : public BT::SyncActionNode
{
public:
  ScriptedSync(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("script"), BT::InputPort<std::string>("probe")};
  }

  NodeStatus tick() override
  {
    std::string script, name;
    getInput("script", script);
    getInput("probe", name);
    Probe & p = g_probes[name];
    if (++p.ticks > 1000) {
      throw std::runtime_error("ScriptedSync '" + name + "' chamada mais de 1000 vezes");
    }
    const size_t idx = std::min<size_t>(p.ticks - 1, script.size() - 1);
    return charToStatus(script[idx]);
  }
};

// Condição controlada por uma chave booleana da blackboard.
class IsTrue : public BT::ConditionNode
{
public:
  IsTrue(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {return {BT::InputPort<bool>("value")};}

  NodeStatus tick() override
  {
    bool v = false;
    getInput("value", v);
    return v ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }
};

// Registra o resultado de getInput de uma porta sem padrão e de uma com padrão "{y}".
bool g_x_ok = false;
bool g_y_ok = false;
double g_y_value = 0.0;

class NeedsInput : public BT::SyncActionNode
{
public:
  NeedsInput(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("x"), BT::InputPort<double>("y", "{y}", "com padrão")};
  }

  NodeStatus tick() override
  {
    double x = 0.0;
    g_x_ok = static_cast<bool>(getInput<double>("x", x));
    g_y_ok = static_cast<bool>(getInput<double>("y", g_y_value));
    return NodeStatus::SUCCESS;
  }
};

// Lê e escreve uma chave inteira numa subárvore.
int g_read_value = -1;

class ReadInt : public BT::SyncActionNode
{
public:
  ReadInt(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {return {BT::InputPort<int>("key")};}

  NodeStatus tick() override
  {
    int v = -1;
    if (!getInput<int>("key", v)) {
      return NodeStatus::FAILURE;
    }
    g_read_value = v;
    return NodeStatus::SUCCESS;
  }
};

class WriteInt : public BT::SyncActionNode
{
public:
  WriteInt(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {return {BT::OutputPort<int>("key")};}

  NodeStatus tick() override
  {
    setOutput("key", 9);
    return NodeStatus::SUCCESS;
  }
};

class BtSemantics : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    std::cout << "[ harness ] BehaviorTree.CPP " << BT::LibraryVersionString() << std::endl;
  }

  void SetUp() override
  {
    g_probes.clear();
    g_x_ok = g_y_ok = false;
    g_read_value = -1;
    factory_.registerNodeType<Scripted>("Scripted");
    factory_.registerNodeType<ScriptedSync>("ScriptedSync");
    factory_.registerNodeType<IsTrue>("IsTrue");
    factory_.registerNodeType<NeedsInput>("NeedsInput");
    factory_.registerNodeType<ReadInt>("ReadInt");
    factory_.registerNodeType<WriteInt>("WriteInt");
  }

  // Cria uma árvore a partir do corpo da árvore principal (e, opcionalmente, de outras).
  BT::Tree make(
    const std::string & body, const BT::Blackboard::Ptr & bb = nullptr,
    const std::string & extra_trees = "")
  {
    const std::string xml =
      R"(<root BTCPP_format="4" main_tree_to_execute="Main"><BehaviorTree ID="Main">)" + body +
      "</BehaviorTree>" + extra_trees + "</root>";
    return factory_.createTreeFromText(xml, bb ? bb : BT::Blackboard::create());
  }

  BT::BehaviorTreeFactory factory_;
};

// --- Sequence com memória x ReactiveSequence ------------------------------------------------

TEST_F(BtSemantics, SequenceComMemoria_NaoReavaliaFilhoAnterior)
{
  auto tree = make(
    R"(<Sequence>
      <ScriptedSync script="S" probe="guarda"/>
      <Scripted script="RRS" probe="acao"/>
    </Sequence>)");
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(g_probes["guarda"].ticks, 1) << "Sequence deveria ter memória e não reavaliar";
}

TEST_F(BtSemantics, ReactiveSequence_ReavaliaFilhoAnteriorEmTodoTick)
{
  auto tree = make(
    R"(<ReactiveSequence>
      <ScriptedSync script="S" probe="guarda"/>
      <Scripted script="RRS" probe="acao"/>
    </ReactiveSequence>)");
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(g_probes["guarda"].ticks, 3);
}

TEST_F(BtSemantics, ReactiveSequence_GuardaFalhaHaltaOFilhoEmExecucao)
{
  auto bb = BT::Blackboard::create();
  bb->set("g", true);
  auto tree = make(
    R"(<ReactiveSequence>
      <IsTrue value="{g}"/>
      <Scripted script="R" probe="acao"/>
    </ReactiveSequence>)",
    bb);

  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(g_probes["acao"].starts, 1);
  EXPECT_EQ(g_probes["acao"].halts, 0);

  bb->set("g", false);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::FAILURE);
  EXPECT_EQ(g_probes["acao"].halts, 1) << "a guarda falhou: o filho deve receber halt";

  EXPECT_EQ(tree.tickOnce(), NodeStatus::FAILURE);
  EXPECT_EQ(g_probes["acao"].halts, 1) << "halt não pode se repetir com a guarda ainda falha";

  bb->set("g", true);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(g_probes["acao"].starts, 2) << "ao voltar a guarda, o filho recomeça (onStart)";
}

// --- Fallback com memória x ReactiveFallback ------------------------------------------------

TEST_F(BtSemantics, Fallback_ComMemoria_NaoReavaliaAlternativaAnterior)
{
  auto tree = make(
    R"(<Fallback>
      <ScriptedSync script="F" probe="primeira"/>
      <Scripted script="R" probe="segunda"/>
    </Fallback>)");
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(g_probes["primeira"].ticks, 1);
  EXPECT_EQ(g_probes["segunda"].ticks, 2);
}

TEST_F(BtSemantics, ReactiveFallback_ReavaliaAlternativaAnteriorEmTodoTick)
{
  auto tree = make(
    R"(<ReactiveFallback>
      <ScriptedSync script="F" probe="primeira"/>
      <Scripted script="R" probe="segunda"/>
    </ReactiveFallback>)");
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(g_probes["primeira"].ticks, 2);
}

TEST_F(BtSemantics, ReactiveFallback_PrioridadeMaiorPreemptaEHaltaOFilhoEmExecucao)
{
  auto bb = BT::Blackboard::create();
  bb->set("alta", false);
  auto tree = make(
    R"(<ReactiveFallback>
      <ReactiveSequence>
        <IsTrue value="{alta}"/>
        <Scripted script="R" probe="alta"/>
      </ReactiveSequence>
      <Scripted script="R" probe="baixa"/>
    </ReactiveFallback>)",
    bb);

  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(g_probes["baixa"].starts, 1);

  bb->set("alta", true);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(g_probes["alta"].starts, 1);
  EXPECT_EQ(g_probes["baixa"].halts, 1) << "o ramo de baixa prioridade deve ser interrompido";

  bb->set("alta", false);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(g_probes["alta"].halts, 1);
  EXPECT_EQ(g_probes["baixa"].starts, 2);
}

// --- Parallel -------------------------------------------------------------------------------

TEST_F(BtSemantics, Parallel_NaoReexecutaFilhoJaConcluido)
{
  auto tree = make(
    R"(<Parallel success_count="2" failure_count="1">
      <Scripted script="RS" probe="a"/>
      <Scripted script="RRRS" probe="b"/>
    </Parallel>)");
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);  // a conclui aqui
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::SUCCESS);  // b conclui aqui
  EXPECT_EQ(g_probes["a"].ticks, 2) << "a já tinha concluído e não deve ser chamada de novo";
  EXPECT_EQ(g_probes["b"].ticks, 4);
}

TEST_F(BtSemantics, Parallel_HaltaOsDemaisFilhosAoDecidir)
{
  auto tree = make(
    R"(<Parallel success_count="1" failure_count="1">
      <Scripted script="RRS" probe="rapido"/>
      <Scripted script="R" probe="lento"/>
    </Parallel>)");
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(g_probes["lento"].halts, 1) << "o filho ainda em execução deve receber halt";
}

// --- RetryUntilSuccessful(-1) com filho síncrono que falha ----------------------------------
//
// Achado na 4.10.0: o RetryUntilSuccessful, ao ver o filho síncrono falhar, devolve RUNNING e
// emite um "wake-up". O que acontece depois depende de como a árvore é executada:
//  - tickExactlyOnce(): devolve o controle, então há UMA tentativa por chamada;
//  - tickOnce() (usado pelo strategy_node): re-executa a raiz na mesma chamada enquanto houver
//    wake-up, então repete as tentativas sem devolver o controle até o filho ter sucesso.

TEST_F(BtSemantics, RetryInfinito_FilhoSincronoQueFalha_TickExactlyOnce_UmaTentativaPorChamada)
{
  auto tree = make(
    R"(<RetryUntilSuccessful num_attempts="-1">
      <ScriptedSync script="FFFS" probe="tentativa"/>
    </RetryUntilSuccessful>)");
  for (int i = 1; i <= 3; ++i) {
    EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING) << "chamada " << i;
    EXPECT_EQ(g_probes["tentativa"].ticks, i) << "esperada uma tentativa por chamada";
  }
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
}

TEST_F(BtSemantics, RetryInfinito_FilhoSincronoQueFalha_TickOnce_RepeteNaMesmaChamadaAteOSucesso)
{
  auto tree = make(
    R"(<RetryUntilSuccessful num_attempts="-1">
      <ScriptedSync script="FFFS" probe="tentativa"/>
    </RetryUntilSuccessful>)");
  EXPECT_EQ(tree.tickOnce(), NodeStatus::SUCCESS)
    << "tickOnce() deveria seguir tentando por conta dos wake-ups, até o filho ter sucesso";
  EXPECT_EQ(g_probes["tentativa"].ticks, 4) << "as 4 tentativas ocorrem dentro de UMA chamada";
}

// --- getInput -------------------------------------------------------------------------------

TEST_F(BtSemantics, GetInput_PortaSemAtributoESemPadrao_Falha_MesmoComChaveNaBlackboard)
{
  auto bb = BT::Blackboard::create();
  bb->set("x", 1.0);  // a biblioteca NÃO deve procurar a chave "x" pelo nome da porta
  bb->set("y", 2.0);
  auto tree = make(R"(<NeedsInput/>)", bb);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::SUCCESS);
  EXPECT_FALSE(g_x_ok) << "porta sem atributo e sem padrão deveria falhar";
  EXPECT_TRUE(g_y_ok) << "porta com padrão \"{y}\" deveria ler a chave y";
  EXPECT_DOUBLE_EQ(g_y_value, 2.0);
}

// --- StatefulActionNode ---------------------------------------------------------------------

TEST_F(BtSemantics, StatefulAction_CicloDeVida)
{
  auto tree = make(R"(<Scripted script="RRS" probe="s"/>)");
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(g_probes["s"].starts, 1);
  EXPECT_EQ(g_probes["s"].running, 2);
  EXPECT_EQ(g_probes["s"].halts, 0) << "onHalted não pode ser chamado quando a folha conclui";

  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(g_probes["s"].starts, 2) << "depois de concluir, um novo tick chama onStart de novo";
}

TEST_F(BtSemantics, StatefulAction_HaltDaArvoreChamaOnHaltedUmaUnicaVez)
{
  auto tree = make(R"(<Scripted script="R" probe="s"/>)");
  EXPECT_EQ(tree.tickOnce(), NodeStatus::RUNNING);
  tree.haltTree();
  EXPECT_EQ(g_probes["s"].halts, 1);
  tree.haltTree();
  EXPECT_EQ(g_probes["s"].halts, 1) << "halt repetido numa folha já parada não chama onHalted";
}

// --- SubTree e blackboard -------------------------------------------------------------------

constexpr const char * kChild =
  R"(<BehaviorTree ID="Child"><Sequence>
    <ReadInt key="{parent_key}"/>
    <WriteInt key="{new_key}"/>
  </Sequence></BehaviorTree>)";

TEST_F(BtSemantics, SubTreeSemAutoremap_NaoEnxergaChavesDoPai)
{
  auto bb = BT::Blackboard::create();
  bb->set("parent_key", 7);
  auto tree = make(R"(<SubTree ID="Child"/>)", bb, kChild);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::FAILURE)
    << "a blackboard da subárvore é isolada: parent_key não deveria existir lá";
}

// Achado na 4.10.0: com _autoremap a subárvore enxerga as chaves do pai E também CRIA no pai as
// chaves novas que ela escreve (Blackboard::createEntryImpl delega ao pai). Ou seja, o
// autoremap não é "copia só chaves que já existem": é uma passagem total (exceto chaves
// privadas, que começam com "_").
TEST_F(BtSemantics, SubTreeComAutoremap_EnxergaChavesDoPaiEChavesNovasVazamParaORaiz)
{
  auto bb = BT::Blackboard::create();
  bb->set("parent_key", 7);
  auto tree = make(R"(<SubTree ID="Child" _autoremap="true"/>)", bb, kChild);
  EXPECT_EQ(tree.tickOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(g_read_value, 7) << "parent_key existe no pai e deve ser lida pela subárvore";
  ASSERT_TRUE(tree.rootBlackboard()->getEntry("new_key") != nullptr)
    << "new_key foi escrita pela subárvore e deveria ter sido criada no pai";
  EXPECT_EQ(tree.rootBlackboard()->get<int>("new_key"), 9);
}

}  // namespace
