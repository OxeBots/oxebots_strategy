#include "oxebots_strategy/is_ball_closest_condition.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace oxebots_strategy
{

// Histerese: para assumir a bola precisa estar mais perto por kEnterMarginMm; para largar, o outro robô
// precisa ficar mais perto por kExitMarginMm. Evita alternar quando duas distâncias são quase iguais.
static constexpr double kEnterMarginMm = 30.0;
static constexpr double kExitMarginMm = 50.0;
static constexpr double kShotMinSpeed = 500.0;
static constexpr double kPredMaxAgeS = 0.15;

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
           BT::InputPort<double>("max_goal_dist", 1500.0,
                                 "Distância máxima (em x) da bola até a linha do nosso gol, em mm") };
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

BT::NodeStatus IsBallClosestCondition::tick()
{
  uint32_t robot_id;
  double ball_x, ball_y;
  double max_goal_dist = 1500.0;
  if (!getInput<uint32_t>("robot_id", robot_id) ||
      !getInput<double>("ball_x", ball_x) ||
      !getInput<double>("ball_y", ball_y)) {
    return BT::NodeStatus::FAILURE;
  }
  (void)getInput<double>("max_goal_dist", max_goal_dist);

  double my_goal_x = 0.0;
  auto root = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
  if (!root || !root->get("my_goal_x", my_goal_x)) return BT::NodeStatus::FAILURE;

  oxebots_interfaces::msg::GameData::SharedPtr game_data;
  oxebots_interfaces::msg::BallPrediction::SharedPtr pred;
  rclcpp::Time pred_stamp(0, 0, RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    game_data = last_game_data_;
    pred = last_ball_pred_;
    pred_stamp = last_ball_pred_stamp_;
  }
  if (!game_data) return BT::NodeStatus::FAILURE;

  // Bola longe do nosso gol: não vale a pena o goleiro sair
  if (std::abs(ball_x - my_goal_x) > max_goal_dist) {
    last_status_ = false;
    return BT::NodeStatus::FAILURE;
  }

  // Chute a caminho do nosso gol: quem cuida é a interceptação do GoalLineDefend
  const double dir = (my_goal_x < 0) ? 1.0 : -1.0;
  if (pred && (node_->now() - pred_stamp).seconds() <= kPredMaxAgeS &&
      dir * pred->vx < 0.0 && std::abs(pred->vx) >= kShotMinSpeed &&
      dir * (pred->x - my_goal_x) > 0.0) {
    last_status_ = false;
    return BT::NodeStatus::FAILURE;
  }

  bool found_me = false;
  double my_dist = std::numeric_limits<double>::infinity();
  double other_dist = std::numeric_limits<double>::infinity();

  for (const auto& ally : game_data->robots.allies) {
    const double d = std::hypot(ally.x - ball_x, ally.y - ball_y);
    if (ally.id == robot_id) {
      found_me = true;
      my_dist = d;
    } else {
      other_dist = std::min(other_dist, d);
    }
  }
  for (const auto& enemy : game_data->robots.enemies) {
    other_dist = std::min(other_dist, std::hypot(enemy.x - ball_x, enemy.y - ball_y));
  }
  if (!found_me) return BT::NodeStatus::FAILURE;

  const double margin = last_status_ ? kExitMarginMm : -kEnterMarginMm;
  last_status_ = (my_dist < other_dist + margin);
  return last_status_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace oxebots_strategy
