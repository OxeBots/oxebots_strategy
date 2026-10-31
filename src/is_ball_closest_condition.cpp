#include "oxebots_strategy/is_ball_closest_condition.h"
#include "oxebots_strategy/goalkeeper_blackboard.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace oxebots_strategy
{

// Histerese: para assumir a bola precisa estar mais perto por kEnterMarginMm; para largar, o outro robô
// precisa ficar mais perto por kExitMarginMm. Evita alternar quando duas distâncias são quase iguais.
// Para assumir, a vantagem tem de ser clara (o atacante também vai à bola; uma ida do goleiro que o atacante
// ganha no caminho é só tempo perdido fora da defesa).
static constexpr double kEnterMarginMm = 150.0;
static constexpr double kExitMarginMm = 50.0;
// Já perto da bola o goleiro termina a jogada (chuta) em vez de desistir por causa de um terceiro robô.
static constexpr double kLockInDistMm = 350.0;

// A coleira (fração da metade aliada) tem duas faixas. Até kInnerBandFraction basta ser o mais perto; além
// disso (e até a coleira) só assume com uma folga maior, bola lenta e nenhum adversário perto da bola.
static constexpr double kInnerBandFraction = 0.4;
static constexpr double kOuterExtraAdvantageMm = 300.0;   // ~0,3 s a 1 m/s
static constexpr double kOuterBallMaxSpeed = 300.0;       // mm/s
static constexpr double kOuterOpponentFreeMm = 800.0;

// A bola que já se afasta rápido do nosso gol não vale a corrida; o chute ao gol é da interceptação.
static constexpr double kShotMinSpeed = 500.0;
static constexpr double kAwayMinSpeed = 800.0;
static constexpr double kPredMaxAgeS = 0.15;

// Compromisso limitado, só quando a bola é disputada (outro robô a menos de kContestedMm dela): sem concluir
// dentro do orçamento, aborta e espera kCooldownS. O orçamento cresce com a distância até a bola no momento em
// que assume (a uma velocidade de aproximação conservadora), para o goleiro conseguir chegar. Bola parada e
// ninguém por perto não tem limite de tempo: o goleiro vai, chuta e volta.
static constexpr double kCommitMinS = 3.0;
static constexpr double kCommitExtraS = 1.5;
static constexpr double kApproachSpeedMmS = 400.0;
static constexpr double kContestedMm = 1000.0;
static constexpr double kCooldownS = 1.5;
// Se a condição ficou sem ser avaliada (outro estado da árvore assumiu), o compromisso anterior é descartado.
static constexpr double kStaleTickS = 0.3;
// Bola logo atrás da linha do gol ainda conta.
static constexpr double kBehindLineMm = 150.0;

IsBallClosestCondition::IsBallClosestCondition(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node)
{
  game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10,
    std::bind(&IsBallClosestCondition::gameDataCallback, this, std::placeholders::_1));

  ball_pred_sub_ = node_->create_subscription<oxebots_interfaces::msg::BallPrediction>(
    "/ball_predicted", 10,
    std::bind(&IsBallClosestCondition::ballPredictionCallback, this, std::placeholders::_1));
}

BT::PortsList IsBallClosestCondition::providedPorts()
{
  return { BT::InputPort<uint32_t>("robot_id"),
           BT::InputPort<double>("ball_x"),
           BT::InputPort<double>("ball_y"),
           BT::InputPort<double>("leash_fraction", goalkeeper::kDefaultLeashFraction,
                                 "Fração da metade aliada (do gol ao meio-campo) até onde o goleiro busca a bola") };
}

void IsBallClosestCondition::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_game_data_ = msg;
}

void IsBallClosestCondition::ballPredictionCallback(const oxebots_interfaces::msg::BallPrediction::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_ball_pred_ = msg;
  last_ball_pred_stamp_ = node_->now();
}

void IsBallClosestCondition::disengage()
{
  engaged_ = false;
}

BT::NodeStatus IsBallClosestCondition::tick()
{
  uint32_t robot_id;
  double ball_x, ball_y;
  double leash_fraction = goalkeeper::kDefaultLeashFraction;
  if (!getInput<uint32_t>("robot_id", robot_id) ||
      !getInput<double>("ball_x", ball_x) ||
      !getInput<double>("ball_y", ball_y)) {
    return BT::NodeStatus::FAILURE;
  }
  (void)getInput<double>("leash_fraction", leash_fraction);

  goalkeeper::FieldGeometry geo;
  if (!goalkeeper::readFieldGeometry(config(), geo)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "IsBallClosest: aguardando a geometria do campo no Blackboard raiz");
    disengage();
    return BT::NodeStatus::FAILURE;
  }

  oxebots_interfaces::msg::GameData::SharedPtr game_data;
  oxebots_interfaces::msg::BallPrediction::SharedPtr pred;
  rclcpp::Time pred_stamp(0, 0, RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    game_data = last_game_data_;
    pred = last_ball_pred_;
    pred_stamp = last_ball_pred_stamp_;
  }
  if (!game_data) {
    disengage();
    return BT::NodeStatus::FAILURE;
  }

  const rclcpp::Time now = node_->now();
  if (engaged_ && (now - last_tick_).seconds() > kStaleTickS) {
    disengage();   // outro estado da árvore esteve no comando; começa do zero
  }
  last_tick_ = now;

  if (now < cooldown_until_) {
    return BT::NodeStatus::FAILURE;
  }

  // Coleira: quão fundo no campo a bola está, contado a partir da linha do nosso gol
  const double depth = geo.forwardSign() * (ball_x - geo.my_goal_x);
  const double leash = leash_fraction * geo.halfLength();
  if (depth > leash || depth < -kBehindLineMm) {
    disengage();
    return BT::NodeStatus::FAILURE;
  }

  // Predição da bola (chute ao gol, bola que se afasta, bola lenta). Sem predição recente, não há "bola lenta".
  const bool pred_fresh = pred && (now - pred_stamp).seconds() <= kPredMaxAgeS;
  double ball_speed = std::numeric_limits<double>::infinity();
  if (pred_fresh) {
    ball_speed = std::hypot(pred->vx, pred->vy);
    const double toward_goal = -geo.forwardSign() * pred->vx;   // > 0: indo para o nosso gol
    if (toward_goal >= kShotMinSpeed && depth > 0.0) {
      disengage();
      return BT::NodeStatus::FAILURE;   // chute a caminho do gol: é da interceptação
    }
    if (-toward_goal >= kAwayMinSpeed) {
      disengage();
      return BT::NodeStatus::FAILURE;   // bola já se afastando rápido: não vale a corrida
    }
  }

  bool found_me = false;
  double my_dist = std::numeric_limits<double>::infinity();
  double other_dist = std::numeric_limits<double>::infinity();
  double ally_dist = std::numeric_limits<double>::infinity();   // aliado mais perto da bola, sem contar o goleiro
  double enemy_dist = std::numeric_limits<double>::infinity();

  for (const auto& ally : game_data->robots.allies) {
    const double d = std::hypot(ally.x - ball_x, ally.y - ball_y);
    if (ally.id == robot_id) {
      found_me = true;
      my_dist = d;
    } else {
      other_dist = std::min(other_dist, d);
      ally_dist = std::min(ally_dist, d);
    }
  }
  for (const auto& enemy : game_data->robots.enemies) {
    const double d = std::hypot(enemy.x - ball_x, enemy.y - ball_y);
    enemy_dist = std::min(enemy_dist, d);
    other_dist = std::min(other_dist, d);
  }
  if (!found_me) {
    disengage();
    return BT::NodeStatus::FAILURE;
  }

  const double margin = engaged_ ? kExitMarginMm : -kEnterMarginMm;
  bool wants = (my_dist < other_dist + margin);

  // Só sai da área se a bola estiver ameaçada: um adversário mais perto dela que qualquer aliado (ou nenhum
  // aliado ativo, e o goleiro é o único que pode ir). Com a bola mais perto de um aliado, quem joga é o aliado.
  const bool enemy_has_it = enemy_dist < ally_dist;
  const bool no_allies = std::isinf(ally_dist);
  if (!(enemy_has_it || no_allies)) {
    wants = false;
  }

  // Faixa externa da coleira: só assume (não precisa para continuar) com folga maior, bola lenta e sem adversário perto
  if (wants && !engaged_ && depth > kInnerBandFraction * geo.halfLength()) {
    wants = (my_dist + kOuterExtraAdvantageMm < other_dist) &&
            (ball_speed <= kOuterBallMaxSpeed) &&
            (enemy_dist >= kOuterOpponentFreeMm);
  }

  if (engaged_ && my_dist < kLockInDistMm) {
    wants = true;
  }

  if (!wants) {
    disengage();
    return BT::NodeStatus::FAILURE;
  }

  if (!engaged_) {
    engaged_ = true;
    engaged_since_ = now;
    commit_budget_s_ = std::max(kCommitMinS, my_dist / kApproachSpeedMmS + kCommitExtraS);
    RCLCPP_INFO(node_->get_logger(),
                "Goleiro assume a bola: dist=%.0fmm (aliado=%.0fmm, inimigo=%.0fmm), profundidade=%.0fmm de %.0fmm, orcamento=%.1fs",
                my_dist, ally_dist, enemy_dist, depth, leash, commit_budget_s_);
  } else if (my_dist >= kLockInDistMm && other_dist < kContestedMm &&
             (now - engaged_since_).seconds() > commit_budget_s_) {
    RCLCPP_INFO(node_->get_logger(),
                "Goleiro desiste da bola disputada (tempo limite de %.1fs): volta à defesa", commit_budget_s_);
    disengage();
    cooldown_until_ = now + rclcpp::Duration::from_seconds(kCooldownS);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

} // namespace oxebots_strategy
