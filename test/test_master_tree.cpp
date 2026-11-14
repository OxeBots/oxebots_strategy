// T05 — Testes da Master Tree (MainTreeEL) com blackboard mock e folhas falsas.
//
// Carrega a árvore REAL (behavior_trees/master_strategy_el.xml e todas as subárvores), com as
// condições de guarda reais e folhas falsas (master_tree_harness.hpp), e verifica, sem ROS ativo e
// sem robôs: qual ramo da raiz decide em cada fase, o que roda dentro dele, a preempção entre ramos
// (halt das folhas interrompidas) e a independência de cor e de lado.
//
// Cenários do Cap. 23 cobertos aqui, no nível da árvore: SC-01, SC-02, SC-03 (ramo e limite
// pedido), SC-04, SC-05, SC-06, SC-07, SC-08, SC-09, SC-10, SC-11, SC-13, SC-14, SC-16 e SC-17.
// A velocidade medida em /robot_commands (SC-02/03) é do T06 e do T08.
//
// Ramos ainda provisórios (Halt "not_implemented"): RunBranch e o ball placement nosso. Os testes os
// FIXAM como "para com segurança": quando A06 ou E04 trocarem o XML, eles falham de propósito e devem
// ser substituídos pelo teste do comportamento real (SC-15/16/17 em RUN, SC-12). O pênalti (E06) já
// é real e tem os testes de SC-09 e SC-10 abaixo.
#include <gtest/gtest.h>

#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "master_tree_harness.hpp"

using namespace master_tree_test;  // NOLINT: harness, Scenario e constantes de fase (kHalt, kStop...)
using BT::NodeStatus;

namespace
{

constexpr const char * kRunBranch = "RunBranch";

class MasterTreeTest : public ::testing::Test
{
protected:
  void SetUp() override {reset();}

  void reset() {h = std::make_unique<MasterTreeHarness>();}

  Scenario & s() {return h->scenario;}

  // Papel do robô. O refresh() NUNCA grava role="kicker": o cobrador é um robô de linha (atacante ou
  // defensor) com is_kicker/kicker_id ligados, e o HasRole("kicker") olha essa flag. "kicker" aqui é
  // atalho para "atacante e cobrador" (o contrato está em test_strategy_node.cpp).
  void useRole(const std::string & role)
  {
    const bool kicker = (role == "kicker");
    s().role = kicker ? "attacker" : role;
    s().is_kicker = kicker;
    s().kicker_id = kicker ? s().robot_id : 999u;
  }

  void goTo(int phase, bool favored, const std::string & role = "idle")
  {
    s().phase = phase;
    s().we_favored = favored;
    useRole(role);
  }

  // Um tick e o ramo que o decidiu.
  std::string tickBranch()
  {
    h->tick();
    return h->branch();
  }

  // Resumo do que rodou no último tick: folhas + slot pedido ao ComputeLegalSlot.
  std::string signature() const
  {
    std::ostringstream out;
    out << "branch=" << h->branch() << " ran=[";
    for (const auto & id : h->ranSet()) {
      out << id << ",";
    }
    out << "] slot=" << h->rawPort("ComputeLegalSlot", "slot") << "/"
        << h->rawPort("ComputeLegalSlot", "index");
    return out.str();
  }

  std::unique_ptr<MasterTreeHarness> h;
};

// ============================================================================
// Estrutura da raiz e seleção do ramo por fase (ramos 0 a 11)
// ============================================================================

TEST_F(MasterTreeTest, RootHasTwelveBranchesInPriorityOrder)
{
  const std::vector<std::string> expected{
    "SafetyGuard", "HaltBranch", "TimeoutBranch", "StopBranch", "BallPlacementBranch",
    "PrepareKickoffBranch", "KickoffBranch", "PreparePenaltyBranch", "PenaltyBranch",
    "FreeKickBranch", "RunBranch", "Halt"};
  EXPECT_EQ(h->rootBranchNames(), expected);
}

struct PhaseCase
{
  int phase;
  bool favored;
  const char * branch;
  const char * name;
};

class MasterTreePhaseTest : public MasterTreeTest,
  public ::testing::WithParamInterface<PhaseCase> {};

TEST_P(MasterTreePhaseTest, PhaseSelectsBranch)
{
  const PhaseCase & c = GetParam();
  goTo(c.phase, c.favored);
  EXPECT_EQ(tickBranch(), c.branch);
}

INSTANTIATE_TEST_SUITE_P(
  AllPhases, MasterTreePhaseTest,
  ::testing::Values(
    PhaseCase{kHalt, false, "HaltBranch", "Halt"},
    PhaseCase{kTimeout, false, "TimeoutBranch", "Timeout"},
    PhaseCase{kStop, false, "StopBranch", "Stop"},
    PhaseCase{kBallPlacement, false, "BallPlacementBranch", "BallPlacementTheirs"},
    PhaseCase{kBallPlacement, true, "BallPlacementBranch", "BallPlacementOurs"},
    PhaseCase{kPrepareKickoff, true, "PrepareKickoffBranch", "PrepareKickoffOurs"},
    PhaseCase{kPrepareKickoff, false, "PrepareKickoffBranch", "PrepareKickoffTheirs"},
    PhaseCase{kKickoff, true, "KickoffBranch", "KickoffOurs"},
    PhaseCase{kKickoff, false, "KickoffBranch", "KickoffTheirs"},
    PhaseCase{kPreparePenalty, true, "PreparePenaltyBranch", "PreparePenaltyOurs"},
    PhaseCase{kPreparePenalty, false, "PreparePenaltyBranch", "PreparePenaltyTheirs"},
    PhaseCase{kPenalty, true, "PenaltyBranch", "PenaltyOurs"},
    PhaseCase{kPenalty, false, "PenaltyBranch", "PenaltyTheirs"},
    PhaseCase{kFreeKick, true, "FreeKickBranch", "FreeKickOurs"},
    PhaseCase{kFreeKick, false, "FreeKickBranch", "FreeKickTheirs"},
    PhaseCase{kRun, false, "RunBranch", "Run"},
    PhaseCase{-1, false, "Halt", "UnknownNegative"},
    PhaseCase{10, false, "Halt", "UnknownTen"},
    PhaseCase{255, false, "Halt", "Unknown255"}),
  [](const ::testing::TestParamInfo<PhaseCase> & info) {return std::string(info.param.name);});

TEST_F(MasterTreeTest, MissingPhaseKeyIsUnknownPhaseAndHalts)
{
  h->omit("phase");
  EXPECT_EQ(tickBranch(), "Halt");
  EXPECT_TRUE(h->ran("Halt:unknown_phase"));
  EXPECT_TRUE(h->runningLeaves().empty());
}

TEST_F(MasterTreeTest, UnknownPhaseNeverMovesTheRobot)
{
  for (const int phase : {-1, 10, 255}) {
    goTo(phase, true, "kicker");
    h->tick();
    EXPECT_TRUE(h->ran("Halt:unknown_phase")) << "fase " << phase;
    EXPECT_TRUE(h->runningLeaves().empty()) << "fase " << phase;
  }
}

// ============================================================================
// Ramo 0: SafetyGuard (SC-01, SC-20 no nível da árvore)
// ============================================================================

TEST_F(MasterTreeTest, FreshDataLeavesSafetyGuardInactive)
{
  goTo(kRun, false);
  EXPECT_EQ(tickBranch(), kRunBranch);
}

TEST_F(MasterTreeTest, StalePoseBallOrGcActivatesSafetyGuard)
{
  struct Stale
  {
    const char * what;
    double Scenario::* age;
    double value;
  };
  const std::vector<Stale> cases{
    {"pose", &Scenario::pose_age_ms, 151.0},
    {"bola", &Scenario::ball_age_ms, 301.0},
    {"GC", &Scenario::gc_age_ms, 501.0},
    {"pose ausente (robô não visto)", &Scenario::pose_age_ms,
      std::numeric_limits<double>::infinity()},
    {"GC stale (inf)", &Scenario::gc_age_ms, std::numeric_limits<double>::infinity()},
  };
  for (const auto & c : cases) {
    reset();
    goTo(kRun, false, "attacker");
    s().*(c.age) = c.value;
    EXPECT_EQ(tickBranch(), "SafetyGuard") << c.what;
    EXPECT_TRUE(h->ran("Halt:stale_data")) << c.what;
    EXPECT_TRUE(h->runningLeaves().empty()) << c.what;
  }
}

TEST_F(MasterTreeTest, AgeLimitsAreInclusive)
{
  goTo(kRun, false);
  s().pose_age_ms = 150.0;
  s().ball_age_ms = 300.0;
  s().gc_age_ms = 500.0;
  EXPECT_EQ(tickBranch(), kRunBranch);

  s().pose_age_ms = 150.5;
  EXPECT_EQ(tickBranch(), "SafetyGuard");
  s().pose_age_ms = 150.0;
  s().ball_age_ms = 300.5;
  EXPECT_EQ(tickBranch(), "SafetyGuard");
  s().ball_age_ms = 300.0;
  s().gc_age_ms = 500.5;
  EXPECT_EQ(tickBranch(), "SafetyGuard");
}

TEST_F(MasterTreeTest, WithoutRequiredGcStaleGcIsIgnored)
{
  goTo(kRun, false);
  s().require_gc = false;
  s().gc_age_ms = std::numeric_limits<double>::infinity();
  EXPECT_EQ(tickBranch(), kRunBranch);

  s().require_gc = true;
  EXPECT_EQ(tickBranch(), "SafetyGuard");
}

TEST_F(MasterTreeTest, StaleDataBeatsEveryPhaseIncludingHalt)
{
  for (const int phase : {kHalt, kTimeout, kStop, kKickoff, kFreeKick, kRun}) {
    reset();
    goTo(phase, true, "kicker");
    s().ball_age_ms = 1000.0;
    EXPECT_EQ(tickBranch(), "SafetyGuard") << "fase " << phase;
    EXPECT_TRUE(h->ran("Halt:stale_data")) << "fase " << phase;
  }
}

TEST_F(MasterTreeTest, DataRecoveryRestartsTheLeafInsteadOfResumingIt)
{
  goTo(kPrepareKickoff, false, "defender");
  EXPECT_EQ(tickBranch(), "PrepareKickoffBranch");
  ASSERT_EQ(h->stats("GoToPoint").starts, 1);

  s().pose_age_ms = 400.0;
  EXPECT_EQ(tickBranch(), "SafetyGuard");
  EXPECT_EQ(h->stats("GoToPoint").halts, 1);

  s().pose_age_ms = 10.0;
  EXPECT_EQ(tickBranch(), "PrepareKickoffBranch");
  EXPECT_EQ(h->stats("GoToPoint").starts, 2);  // onStart de novo: recalcula o alvo com dado novo
}

// ============================================================================
// Ramos 1 e 2: HALT e TIMEOUT (SC-02, SC-13)
// ============================================================================

TEST_F(MasterTreeTest, HaltPhaseRunsHaltSubTreeOnly)
{
  goTo(kHalt, true, "kicker");
  EXPECT_EQ(tickBranch(), "HaltBranch");
  EXPECT_TRUE(h->ran("Halt:gc_halt"));
  EXPECT_TRUE(h->runningLeaves().empty());
}

TEST_F(MasterTreeTest, TimeoutPhaseRunsTimeoutTreeOnly)
{
  goTo(kTimeout, true, "kicker");
  EXPECT_EQ(tickBranch(), "TimeoutBranch");
  EXPECT_TRUE(h->ran("Halt:timeout"));
  EXPECT_TRUE(h->runningLeaves().empty());
}

struct Origin
{
  const char * name;
  int phase;
  bool favored;
  const char * role;
};

class MasterTreeHaltPreemptionTest : public MasterTreeTest,
  public ::testing::WithParamInterface<Origin> {};

// SC-02: de qualquer ramo com uma ação em andamento, o HALT interrompe (halt) a ação no mesmo tick.
TEST_P(MasterTreeHaltPreemptionTest, HaltHaltsTheRunningLeafInTheSameTick)
{
  const Origin & o = GetParam();
  goTo(o.phase, o.favored, o.role);
  h->tick();
  const std::set<std::string> running = h->runningLeaves();
  ASSERT_FALSE(running.empty()) << "o cenário de origem precisa ter uma ação em andamento";
  const int halts_before = h->totalHalts();

  s().phase = kHalt;
  h->tick();

  EXPECT_EQ(h->branch(), "HaltBranch");
  EXPECT_TRUE(h->ran("Halt:gc_halt"));
  EXPECT_EQ(h->totalHalts() - halts_before, static_cast<int>(running.size()));
  for (const auto & id : running) {
    EXPECT_FALSE(h->ran(id)) << id << " não pode rodar no tick do HALT";
  }
  EXPECT_TRUE(h->runningLeaves().empty());
}

INSTANTIATE_TEST_SUITE_P(
  FromEveryBranch, MasterTreeHaltPreemptionTest,
  ::testing::Values(
    Origin{"StopDefender", kStop, false, "defender"},
    Origin{"StopGoalkeeper", kStop, false, "goalkeeper"},
    Origin{"BallPlacementTheirs", kBallPlacement, false, "defender"},
    Origin{"PrepareKickoffOursKicker", kPrepareKickoff, true, "kicker"},
    Origin{"PrepareKickoffTheirsDefender", kPrepareKickoff, false, "defender"},
    Origin{"KickoffOursKicker", kKickoff, true, "kicker"},
    Origin{"KickoffOursOther", kKickoff, true, "defender"},
    Origin{"KickoffTheirsAttacker", kKickoff, false, "attacker"},
    Origin{"FreeKickOursKicker", kFreeKick, true, "kicker"},
    Origin{"FreeKickOursSupport", kFreeKick, true, "attacker"},
    Origin{"FreeKickTheirsDefender", kFreeKick, false, "defender"},
    Origin{"PrepareKickoffGoalkeeper", kPrepareKickoff, true, "goalkeeper"},
    Origin{"PreparePenaltyOursKicker", kPreparePenalty, true, "kicker"},
    Origin{"PreparePenaltyTheirsGoalkeeper", kPreparePenalty, false, "goalkeeper"},
    Origin{"PenaltyOursKicker", kPenalty, true, "kicker"},
    Origin{"PenaltyTheirsOther", kPenalty, false, "defender"}),
  [](const ::testing::TestParamInfo<Origin> & info) {return std::string(info.param.name);});

// ============================================================================
// Ramo 3: STOP (SC-03, SC-14)
// ============================================================================

struct StopCase
{
  const char * name;
  const char * role;
  bool ball_close;
  const char * slot;      // slot pedido ao ComputeLegalSlot ("" = nenhum)
  const char * leaf;      // folha de movimento esperada
};

class MasterTreeStopTest : public MasterTreeTest,
  public ::testing::WithParamInterface<StopCase> {};

TEST_P(MasterTreeStopTest, StopDispatchesByRoleAndBallDistance)
{
  const StopCase & c = GetParam();
  goTo(kStop, false, c.role);
  h->setStatus("IsBallClose", c.ball_close ? NodeStatus::SUCCESS : NodeStatus::FAILURE);

  EXPECT_EQ(tickBranch(), "StopBranch");
  EXPECT_EQ(h->rawPort("SetMotionLimit", "limit_mps"), "0.75");  // SC-03: teto pedido ao SafetyGate
  EXPECT_TRUE(h->ran(c.leaf)) << signature();
  if (std::string(c.slot).empty()) {
    EXPECT_FALSE(h->ran("ComputeLegalSlot")) << signature();
  } else {
    EXPECT_TRUE(h->ran("ComputeLegalSlot")) << signature();
    EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), c.slot);
  }
}

INSTANTIATE_TEST_SUITE_P(
  Roles, MasterTreeStopTest,
  ::testing::Values(
    StopCase{"GoalkeeperFar", "goalkeeper", false, "", "GoalLineDefend"},
    StopCase{"AttackerFar", "attacker", false, "stop_shadow", "GoToPoint"},
    StopCase{"DefenderFar", "defender", false, "stop_def", "GoToPoint"},
    StopCase{"IdleFar", "idle", false, "", "HoldPosition"},
    // Bola a < 650 mm: QUALQUER papel (inclusive goleiro) recua para o ponto legal.
    StopCase{"GoalkeeperClose", "goalkeeper", true, "retreat", "GoToPoint"},
    StopCase{"AttackerClose", "attacker", true, "retreat", "GoToPoint"},
    StopCase{"DefenderClose", "defender", true, "retreat", "GoToPoint"},
    StopCase{"IdleClose", "idle", true, "retreat", "GoToPoint"}),
  [](const ::testing::TestParamInfo<StopCase> & info) {return std::string(info.param.name);});

TEST_F(MasterTreeTest, StopHaltsTheOldGoalInsteadOfLettingItPersist)
{
  // SC-14: o robô vai a um ponto (GoToPoint do pré-kick-off) quando chega o STOP.
  goTo(kPrepareKickoff, false, "defender");
  ASSERT_EQ(tickBranch(), "PrepareKickoffBranch");
  ASSERT_TRUE(h->ran("GoToPoint"));

  goTo(kStop, false, "idle");  // longe da bola: HoldPosition
  EXPECT_EQ(tickBranch(), "StopBranch");
  EXPECT_EQ(h->stats("GoToPoint").halts, 1);  // o objetivo antigo recebeu halt
  EXPECT_FALSE(h->ran("GoToPoint"));
  EXPECT_TRUE(h->ran("HoldPosition"));
}

TEST_F(MasterTreeTest, StopThenForceStartResumesTheRolesAfterStop)
{
  // SC-04: STOP -> FORCE_START (fase RUN). O ramo de STOP é interrompido no mesmo tick.
  goTo(kStop, false, "idle");
  ASSERT_EQ(tickBranch(), "StopBranch");
  ASSERT_TRUE(h->ran("HoldPosition"));

  goTo(kRun, false, "idle");
  EXPECT_EQ(tickBranch(), kRunBranch);
  EXPECT_EQ(h->stats("HoldPosition").halts, 1);
  EXPECT_FALSE(h->ran("HoldPosition"));
}

// ============================================================================
// Ramo 4: BALL_PLACEMENT (SC-11; o nosso é E04, ainda provisório)
// ============================================================================

TEST_F(MasterTreeTest, BallPlacementTheirsLimitsSpeedAndHoldsOutsideTheCorridor)
{
  goTo(kBallPlacement, false, "defender");
  EXPECT_EQ(tickBranch(), "BallPlacementBranch");
  EXPECT_EQ(h->rawPort("SetMotionLimit", "limit_mps"), "0.75");
  EXPECT_TRUE(h->ran("AvoidPlacementCorridor"));
  EXPECT_TRUE(h->ran("HoldPosition"));  // fora do corredor: fica parado
}

TEST_F(MasterTreeTest, BallPlacementTheirsMovesOutOfTheCorridorBeforeHolding)
{
  goTo(kBallPlacement, false, "defender");
  h->setStatus("AvoidPlacementCorridor", NodeStatus::RUNNING);  // ainda dentro do corredor
  EXPECT_EQ(tickBranch(), "BallPlacementBranch");
  EXPECT_TRUE(h->ran("AvoidPlacementCorridor"));
  EXPECT_FALSE(h->ran("HoldPosition"));
  EXPECT_FALSE(h->ran("GoToPoint"));

  h->setStatus("AvoidPlacementCorridor", NodeStatus::SUCCESS);  // saiu
  h->tick();
  EXPECT_TRUE(h->ran("HoldPosition"));
}

TEST_F(MasterTreeTest, BallPlacementOursIsStillAnExplicitSafeHalt)
{
  // E04 ainda não existe: o ramo para com o motivo registrado. Substituir ao implementar o E04.
  goTo(kBallPlacement, true, "kicker");
  EXPECT_EQ(tickBranch(), "BallPlacementBranch");
  EXPECT_TRUE(h->ran("Halt:ball_placement_our_not_implemented"));
  EXPECT_TRUE(h->runningLeaves().empty());
}

// ============================================================================
// Ramos 5 e 6: kick-off (SC-05, SC-06) e independência de cor e de lado (D04)
// ============================================================================

TEST_F(MasterTreeTest, PrepareKickoffOursDispatchesGoalkeeperKickerAndLine)
{
  goTo(kPrepareKickoff, true, "goalkeeper");
  EXPECT_EQ(tickBranch(), "PrepareKickoffBranch");
  EXPECT_TRUE(h->ran("GoalLineDefend"));
  EXPECT_FALSE(h->ran("ComputeLegalSlot"));

  goTo(kPrepareKickoff, true, "kicker");
  h->tick();
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "kickoff_kicker");
  EXPECT_TRUE(h->ran("GoToPoint"));

  for (const char * role : {"attacker", "defender", "idle"}) {
    goTo(kPrepareKickoff, true, role);
    h->tick();
    EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "kickoff_def") << role;
  }
}

TEST_F(MasterTreeTest, KickerFlagWinsOverTheAttackerRoleString)
{
  goTo(kPrepareKickoff, true, "attacker");
  h->tick();
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "kickoff_def");  // atacante sem a flag

  s().is_kicker = true;  // o atribuidor trava este robô como cobrador do reinício
  s().kicker_id = s().robot_id;
  h->tick();
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "kickoff_kicker");

  // Só o kicker_id (sem is_kicker) também vale, como o HasRole aceita.
  s().is_kicker = false;
  h->tick();
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "kickoff_kicker");
}

TEST_F(MasterTreeTest, PrepareKickoffTheirsKeepsEveryoneOnTheirOwnHalfSlots)
{
  goTo(kPrepareKickoff, false, "attacker");
  EXPECT_EQ(tickBranch(), "PrepareKickoffBranch");
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "kickoff_def");
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "index"), "0");

  goTo(kPrepareKickoff, false, "defender");
  h->tick();
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "kickoff_def");
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "index"), "1");

  // No kick-off do adversário ninguém ocupa o slot do cobrador.
  for (const char * role : {"attacker", "defender", "idle", "kicker"}) {
    goTo(kPrepareKickoff, false, role);
    h->tick();
    EXPECT_NE(h->rawPort("ComputeLegalSlot", "slot"), "kickoff_kicker") << role;
  }
}

TEST_F(MasterTreeTest, OnlyTheKickerTouchesTheBallInOurKickoff)
{
  // Tudo "favorável" ao chute: chegou ao pré-chute e a bola está ao alcance.
  h->setStatus("GoToPoint", NodeStatus::SUCCESS);
  h->setStatus("IsBallClose", NodeStatus::SUCCESS);

  goTo(kKickoff, true, "kicker");
  EXPECT_EQ(tickBranch(), "KickoffBranch");
  EXPECT_TRUE(h->ran("KickOnce"));
  EXPECT_TRUE(h->ran("UpdateBallPosition"));

  for (const char * role : {"attacker", "defender", "idle"}) {
    reset();
    h->setStatus("GoToPoint", NodeStatus::SUCCESS);
    h->setStatus("IsBallClose", NodeStatus::SUCCESS);
    goTo(kKickoff, true, role);
    EXPECT_EQ(tickBranch(), "KickoffBranch") << role;
    EXPECT_FALSE(h->ran("KickOnce")) << role;  // R-12
    EXPECT_TRUE(h->ran("HoldPosition")) << role;
  }
}

TEST_F(MasterTreeTest, TheirKickoffNeverKicksAndWaitsInTheirOwnHalf)
{
  for (const char * role : {"goalkeeper", "kicker", "attacker", "defender", "idle"}) {
    reset();
    h->setStatus("GoToPoint", NodeStatus::SUCCESS);
    h->setStatus("IsBallClose", NodeStatus::SUCCESS);
    goTo(kKickoff, false, role);
    EXPECT_EQ(tickBranch(), "KickoffBranch") << role;
    EXPECT_FALSE(h->ran("KickOnce")) << role;  // SC-06
    EXPECT_FALSE(h->ran("UpdateBallPosition") && std::string(role) != "goalkeeper") << role;
  }
}

TEST_F(MasterTreeTest, KickoffAndFreeKickTreesDoNotDependOnColorOrSide)
{
  // D04 / SC-05 "repetir para amarelo e azul": o mesmo cenário lógico dá a mesma execução com
  // is_yellow e side_sign trocados (a árvore não tem coordenada literal nem lê a cor).
  struct Case
  {
    const char * label;
    int phase;
    bool favored;
    const char * role;
  };
  const std::vector<Case> cases{
    {"prep_our_kicker", kPrepareKickoff, true, "kicker"},
    {"prep_our_other", kPrepareKickoff, true, "defender"},
    {"prep_their_attacker", kPrepareKickoff, false, "attacker"},
    {"prep_their_defender", kPrepareKickoff, false, "defender"},
    {"kickoff_our_kicker", kKickoff, true, "kicker"},
    {"kickoff_our_other", kKickoff, true, "attacker"},
    {"kickoff_their", kKickoff, false, "defender"},
    {"freekick_our_kicker", kFreeKick, true, "kicker"},
    {"freekick_our_support", kFreeKick, true, "attacker"},
    {"freekick_their_defender", kFreeKick, false, "defender"},
    {"freekick_their_attacker", kFreeKick, false, "attacker"},
    {"stop_attacker", kStop, false, "attacker"},
    {"penalty_prep_our_kicker", kPreparePenalty, true, "kicker"},
    {"penalty_prep_our_defender", kPreparePenalty, true, "defender"},
    {"penalty_prep_their_goalkeeper", kPreparePenalty, false, "goalkeeper"},
    {"penalty_prep_their_attacker", kPreparePenalty, false, "attacker"},
    {"penalty_exec_our_kicker", kPenalty, true, "kicker"},
    {"penalty_exec_their_goalkeeper", kPenalty, false, "goalkeeper"},
    {"penalty_exec_their_other", kPenalty, false, "defender"},
  };

  std::map<std::string, std::string> baseline;
  for (const bool yellow : {false, true}) {
    for (const int side : {1, -1}) {
      for (const auto & c : cases) {
        reset();
        s().is_yellow = yellow;
        s().side_sign = side;
        goTo(c.phase, c.favored, c.role);
        h->tick();
        const std::string sig = signature();
        if (yellow == false && side == 1) {
          baseline[c.label] = sig;
        } else {
          EXPECT_EQ(sig, baseline[c.label])
            << c.label << " com is_yellow=" << yellow << " side_sign=" << side;
        }
      }
    }
  }
}

// ============================================================================
// Ramo 9: free kick (SC-07, SC-08)
// ============================================================================

TEST_F(MasterTreeTest, FreeKickOursKickerKicksAtTheOpponentGoalAndSupportKeepsLegalSpot)
{
  goTo(kFreeKick, true, "kicker");
  EXPECT_EQ(tickBranch(), "FreeKickBranch");
  EXPECT_TRUE(h->ran("UpdateBallPosition"));  // RestartKickerTree
  EXPECT_FALSE(h->ran("ComputeLegalSlot"));   // falta direta: sem jogada ensaiada
  h->setStatus("GoToPoint", NodeStatus::SUCCESS);
  h->setStatus("IsBallClose", NodeStatus::SUCCESS);
  h->tick();
  EXPECT_TRUE(h->ran("KickOnce"));

  goTo(kFreeKick, true, "attacker");
  h->tick();
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "freekick_support");

  goTo(kFreeKick, true, "goalkeeper");
  h->tick();
  EXPECT_TRUE(h->ran("GoalLineDefend"));
}

TEST_F(MasterTreeTest, FreeKickOursCornerAndThrowInPickTheSetPlayOnBothSides)
{
  for (const int side : {1, -1}) {
    reset();
    s().side_sign = side;
    const double opp_goal_x = -s().ownGoalX();

    // Escanteio ofensivo: bola perto da linha de fundo adversária e da lateral.
    s().ball_x = 0.95 * opp_goal_x;
    s().ball_y = 1400.0;
    goTo(kFreeKick, true, "kicker");
    h->tick();
    EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "corner_target") << "side " << side;
    goTo(kFreeKick, true, "attacker");
    h->tick();
    EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "corner_receiver") << "side " << side;

    // Lateral: bola junto da lateral, longe das linhas de fundo.
    s().ball_x = 0.0;
    s().ball_y = 1300.0;
    goTo(kFreeKick, true, "kicker");
    h->tick();
    EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "throwin_target") << "side " << side;
    goTo(kFreeKick, true, "attacker");
    h->tick();
    EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "throwin_receiver") << "side " << side;
  }
}

TEST_F(MasterTreeTest, FreeKickTheirsFormsWallAndNeverKicks)
{
  goTo(kFreeKick, false, "defender");
  EXPECT_EQ(tickBranch(), "FreeKickBranch");
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "wall");

  goTo(kFreeKick, false, "attacker");
  h->tick();
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "def_box_guard");

  goTo(kFreeKick, false, "idle");
  h->tick();
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "wall");

  goTo(kFreeKick, false, "goalkeeper");
  h->tick();
  EXPECT_TRUE(h->ran("GoalLineDefend"));

  h->setStatus("GoToPoint", NodeStatus::SUCCESS);
  h->setStatus("IsBallClose", NodeStatus::SUCCESS);
  for (const char * role : {"goalkeeper", "kicker", "attacker", "defender", "idle"}) {
    goTo(kFreeKick, false, role);
    h->tick();
    EXPECT_FALSE(h->ran("KickOnce")) << role;  // SC-08
  }
}

TEST_F(MasterTreeTest, FreeKickTheirsBallWithin650mmMakesEveryRoleRetreat)
{
  h->setStatus("IsBallClose", NodeStatus::SUCCESS);
  for (const char * role : {"goalkeeper", "attacker", "defender", "idle"}) {
    goTo(kFreeKick, false, role);
    EXPECT_EQ(tickBranch(), "FreeKickBranch") << role;
    EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "retreat") << role;  // R-28
    EXPECT_TRUE(h->ran("GoToPoint")) << role;
  }
}

TEST_F(MasterTreeTest, MissingFavoredKeyIsTreatedAsNotFavored)
{
  // Seguro por padrão: sem we_favored a árvore NÃO cobra o reinício.
  h->setStatus("GoToPoint", NodeStatus::SUCCESS);
  h->setStatus("IsBallClose", NodeStatus::SUCCESS);
  h->omit("we_favored");
  for (const int phase : {kPrepareKickoff, kKickoff, kFreeKick, kPreparePenalty, kPenalty}) {
    goTo(phase, true, "kicker");
    h->tick();
    EXPECT_FALSE(h->ran("KickOnce")) << "fase " << phase;
    EXPECT_NE(h->rawPort("ComputeLegalSlot", "slot"), "penalty_kicker") << "fase " << phase;
  }
}

// ============================================================================
// Goleiro ausente (SC-17, no nível da árvore): ninguém assume o gol por engano
// ============================================================================

TEST_F(MasterTreeTest, WithoutGoalkeeperRoleNobodyRunsTheGoalkeeperLogic)
{
  const struct
  {
    int phase;
    bool favored;
  } phases[] = {
    {kStop, false}, {kPrepareKickoff, true}, {kPrepareKickoff, false}, {kKickoff, true},
    {kKickoff, false}, {kFreeKick, true}, {kFreeKick, false}, {kPreparePenalty, true},
    {kPreparePenalty, false}, {kPenalty, true}, {kPenalty, false}};
  for (const auto & p : phases) {
    for (const char * role : {"attacker", "defender", "idle"}) {
      reset();
      goTo(p.phase, p.favored, role);
      h->tick();
      EXPECT_FALSE(h->ran("GoalLineDefend")) << "fase " << p.phase << " papel " << role;
      EXPECT_FALSE(h->ran("Goalkeeper")) << "fase " << p.phase << " papel " << role;
      EXPECT_FALSE(h->ran("PenaltyKeeperLine")) << "fase " << p.phase << " papel " << role;
    }
  }
}

// ============================================================================
// Pênalti (E06): SC-09 e SC-10
// ============================================================================

TEST_F(MasterTreeTest, PreparePenaltyOursDispatchesKickerBehindAndGoalkeeper)
{
  goTo(kPreparePenalty, true, "goalkeeper");
  EXPECT_EQ(tickBranch(), "PreparePenaltyBranch");
  EXPECT_TRUE(h->ran("GoalLineDefend"));
  EXPECT_FALSE(h->ran("ComputeLegalSlot"));

  goTo(kPreparePenalty, true, "kicker");
  h->tick();
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "penalty_kicker");  // atrás da bola, sem tocá-la
  EXPECT_TRUE(h->ran("GoToPoint"));
  EXPECT_FALSE(h->ran("KickOnce"));  // só chuta depois do NORMAL_START

  // Demais robôs ficam >= 1 m atrás da bola, cada um no seu índice (R-15).
  goTo(kPreparePenalty, true, "attacker");
  h->tick();
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "penalty_behind");
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "index"), "0");

  goTo(kPreparePenalty, true, "defender");
  h->tick();
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "penalty_behind");
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "index"), "1");

  goTo(kPreparePenalty, true, "idle");
  h->tick();
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "penalty_behind");
}

TEST_F(MasterTreeTest, PreparePenaltyTheirsKeepsGoalkeeperOnTheLineAndOthersBehind)
{
  goTo(kPreparePenalty, false, "goalkeeper");
  EXPECT_EQ(tickBranch(), "PreparePenaltyBranch");
  EXPECT_TRUE(h->ran("PenaltyKeeperLine"));
  EXPECT_EQ(h->rawPort("PenaltyKeeperLine", "side"), "defend");

  for (const char * role : {"attacker", "defender", "idle", "kicker"}) {
    goTo(kPreparePenalty, false, role);
    h->tick();
    EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "penalty_behind") << role;
    EXPECT_FALSE(h->ran("PenaltyKeeperLine")) << role;  // só o goleiro vai à linha
  }
}

TEST_F(MasterTreeTest, OnlyTheKickerTouchesTheBallInOurPenalty)
{
  for (const char * role : {"kicker", "attacker", "defender", "idle"}) {
    reset();
    h->setStatus("GoToPoint", NodeStatus::SUCCESS);
    h->setStatus("IsBallClose", NodeStatus::SUCCESS);
    goTo(kPenalty, true, role);
    EXPECT_EQ(tickBranch(), "PenaltyBranch") << role;
    if (std::string(role) == "kicker") {
      EXPECT_TRUE(h->ran("KickOnce"));  // RestartKickerTree
      EXPECT_TRUE(h->ran("UpdateBallPosition"));
    } else {
      EXPECT_FALSE(h->ran("KickOnce")) << role;  // R-15: os demais ficam parados
      EXPECT_TRUE(h->ran("HoldPosition")) << role;
    }
  }
}

TEST_F(MasterTreeTest, PenaltyTheirsGoalkeeperHoldsTheLineUntilTheBallIsInPlay)
{
  goTo(kPenalty, false, "goalkeeper");
  EXPECT_EQ(tickBranch(), "PenaltyBranch");
  EXPECT_TRUE(h->ran("PenaltyKeeperLine"));  // bola ainda não entrou em jogo
  EXPECT_FALSE(h->ran("GoalLineDefend"));

  h->setStatus("WaitBallInPlay", NodeStatus::SUCCESS);  // bola se moveu 50 mm (ou 10 s)
  h->tick();
  EXPECT_EQ(h->branch(), "PenaltyBranch");
  EXPECT_TRUE(h->ran("GoalLineDefend"));  // volta ao goleiro normal
  EXPECT_FALSE(h->ran("PenaltyKeeperLine"));
  EXPECT_EQ(h->stats("PenaltyKeeperLine").halts, 1);
}

TEST_F(MasterTreeTest, PenaltyTheirsOtherRobotsHoldPositionAndNeverKick)
{
  h->setStatus("GoToPoint", NodeStatus::SUCCESS);
  h->setStatus("IsBallClose", NodeStatus::SUCCESS);
  for (const char * role : {"kicker", "attacker", "defender", "idle"}) {
    goTo(kPenalty, false, role);
    EXPECT_EQ(tickBranch(), "PenaltyBranch") << role;
    EXPECT_TRUE(h->ran("HoldPosition")) << role;
    EXPECT_FALSE(h->ran("KickOnce")) << role;
  }
}

// ============================================================================
// Ramo provisório: fixado como "para com segurança" até o A06
// ============================================================================

TEST_F(MasterTreeTest, RunBranchIsStillAnExplicitSafeHalt)
{
  // Substituir ao implementar o A06 (NormalPlayTree: SC-15/16/17 em RUN).
  for (const bool favored : {true, false}) {
    reset();
    goTo(kRun, favored, "kicker");
    h->tick();
    EXPECT_EQ(h->branch(), kRunBranch) << "favored " << favored;
    EXPECT_TRUE(h->ran("Halt:not_implemented")) << "favored " << favored;
    EXPECT_TRUE(h->runningLeaves().empty()) << "favored " << favored;
  }
}

// ============================================================================
// Preempção, guardas reativas e estabilidade (Cap. 16)
// ============================================================================

TEST_F(MasterTreeTest, BranchGuardIsReevaluatedEveryTick)
{
  // KICKOFF nosso com o cobrador em andamento; a bola entra em jogo e a fase vira RUN.
  goTo(kKickoff, true, "kicker");
  ASSERT_EQ(tickBranch(), "KickoffBranch");
  ASSERT_TRUE(h->ran("GoToPoint"));

  s().phase = kRun;
  EXPECT_EQ(tickBranch(), kRunBranch);
  EXPECT_FALSE(h->ran("GoToPoint"));  // nem um tick a mais do kick-off
  EXPECT_EQ(h->stats("GoToPoint").halts, 1);
}

TEST_F(MasterTreeTest, SteadyStateDoesNotRestartTheLeaf)
{
  goTo(kFreeKick, false, "defender");
  h->tick(30);
  EXPECT_EQ(h->branch(), "FreeKickBranch");
  EXPECT_EQ(h->stats("GoToPoint").starts, 1);
  EXPECT_EQ(h->stats("GoToPoint").halts, 0);
  EXPECT_EQ(h->stats("GoToPoint").ticks, 30);
}

TEST_F(MasterTreeTest, RoleChangeHaltsTheOldRoleBranch)
{
  // SC-16: o atribuidor tira o cobrador do robô no meio da preparação.
  goTo(kPrepareKickoff, true, "kicker");
  ASSERT_EQ(tickBranch(), "PrepareKickoffBranch");
  ASSERT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "kickoff_kicker");

  useRole("defender");
  h->tick();
  EXPECT_EQ(h->branch(), "PrepareKickoffBranch");
  EXPECT_EQ(h->rawPort("ComputeLegalSlot", "slot"), "kickoff_def");
  EXPECT_EQ(h->stats("GoToPoint").halts, 1);  // o GoToPoint do cobrador foi interrompido
  EXPECT_EQ(h->stats("GoToPoint").starts, 2);
}

TEST_F(MasterTreeTest, SequenceOfPhasesFollowsTheExpectedBranches)
{
  // Trajetória 1 do Cap. 16: sem GC -> STOP -> kick-off nosso -> RUN -> STOP -> HALT.
  struct Step
  {
    int phase;
    bool favored;
    double gc_age_ms;
    const char * expected;
  };
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<Step> steps{
    {kHalt, false, inf, "SafetyGuard"},
    {kStop, false, 10.0, "StopBranch"},
    {kPrepareKickoff, true, 10.0, "PrepareKickoffBranch"},
    {kKickoff, true, 10.0, "KickoffBranch"},
    {kRun, false, 10.0, "RunBranch"},
    {kStop, false, 10.0, "StopBranch"},
    {kFreeKick, false, 10.0, "FreeKickBranch"},
    {kHalt, false, 10.0, "HaltBranch"},
  };
  std::vector<std::string> branches;
  for (const auto & step : steps) {
    goTo(step.phase, step.favored, "defender");
    s().gc_age_ms = step.gc_age_ms;
    branches.push_back(tickBranch());
  }
  std::vector<std::string> expected;
  for (const auto & step : steps) {
    expected.push_back(step.expected);
  }
  EXPECT_EQ(branches, expected);
}

TEST_F(MasterTreeTest, HaltTreeLeavesNoActionRunning)
{
  goTo(kFreeKick, true, "kicker");
  h->tick();
  ASSERT_FALSE(h->runningLeaves().empty());
  const int halts_before = h->totalHalts();
  h->haltTree();
  EXPECT_GT(h->totalHalts(), halts_before);
}

}  // namespace
