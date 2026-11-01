#ifndef OXEBOTS_STRATEGY__IS_BALL_CLOSEST_CONDITION_H_
#define OXEBOTS_STRATEGY__IS_BALL_CLOSEST_CONDITION_H_

#include <mutex>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/ball_prediction.hpp"

namespace oxebots_strategy
{

// SUCCESS quando o goleiro deve sair da área para buscar a bola: ela está mais perto dele do que de qualquer
// outro robô (aliado ou adversário), um adversário está mais perto dela que qualquer aliado (bola ameaçada; se a
// bola é de um aliado, quem joga é o aliado), dentro da "coleira" (fração da metade aliada), sem ser um chute a
// caminho do gol nem uma bola que já se afasta rápido. Na faixa externa da coleira exige mais vantagem, bola
// lenta e nenhum adversário por perto. Quando a bola é disputada, o compromisso é limitado no tempo (orçamento
// proporcional à distância): passado o limite, aborta e só aceita novo compromisso depois de uma pausa (o goleiro
// volta à defesa em vez de ficar disputando a bola). Com a bola parada e ninguém por perto não há limite.
class IsBallClosestCondition : public BT::ConditionNode
{
public:
  IsBallClosestCondition(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
  void ballPredictionCallback(const oxebots_interfaces::msg::BallPrediction::SharedPtr msg);
  void disengage();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
  rclcpp::Subscription<oxebots_interfaces::msg::BallPrediction>::SharedPtr ball_pred_sub_;

  std::mutex data_mutex_;
  oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
  oxebots_interfaces::msg::BallPrediction::SharedPtr last_ball_pred_;
  rclcpp::Time last_ball_pred_stamp_{0, 0, RCL_ROS_TIME};

  bool engaged_ = false;
  rclcpp::Time engaged_since_{0, 0, RCL_ROS_TIME};
  double commit_budget_s_ = 0.0;
  rclcpp::Time cooldown_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
};

} // namespace oxebots_strategy

#endif // OXEBOTS_STRATEGY__IS_BALL_CLOSEST_CONDITION_H_
