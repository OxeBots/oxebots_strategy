#pragma once

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include <cstdint>
#include <limits>

// ---------------------------------------------------------------------------
// Tipos compartilhados entre os nós defensivos
// ---------------------------------------------------------------------------
struct Pose2D   { double x, y, theta; };
struct Vector2D { double x, y; };

// ---------------------------------------------------------------------------
// Infraestrutura de controle (publisher singleton para /robot_goal + /game_data)
// ---------------------------------------------------------------------------
void configureDefenderController(const rclcpp::Node::SharedPtr& node, uint32_t robot_id);
void sendToController(
    const Pose2D& target,
    uint8_t planner_type = oxebots_interfaces::msg::RobotGoal::PLANNER_DSTAR,
    bool disable_arrival_latch = false);
void setDefenderGameDataForTest(const oxebots_interfaces::msg::GameData::SharedPtr& msg);

// ---------------------------------------------------------------------------
// Condições
// ---------------------------------------------------------------------------

/// Verifica se a bola está no campo adversário, ou seja, do lado oposto ao nosso gol
/// (porta `my_goal_x`, padrão {my_goal_x} da blackboard). Não depende da cor do time.
class IsBallInOpponentField : public BT::ConditionNode {
public:
    using BT::ConditionNode::ConditionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

/// Verifica se um oponente está com a posse de bola.
class IsOpponentControllingBall : public BT::ConditionNode {
public:
    using BT::ConditionNode::ConditionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

// ---------------------------------------------------------------------------
// Ações
// ---------------------------------------------------------------------------

/// Estágios 1 e 2.2: posiciona o defensor na linha bola -> gol, fora da pequena área.
/// - Em chute rápido contra o nosso gol, intercepta diretamente a reta da trajetória da bola.
/// - Fora de chute, divide a cobertura do gol com o goleiro quando sua pose está disponível.
/// Nunca cruza o meio-campo e nunca entra na pequena área.
class ShadowBall : public BT::SyncActionNode {
public:
    using BT::SyncActionNode::SyncActionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

/// Estágio 2.1: Marca o oponente posicionando-se na linha bola→gol a distância
/// fixa da bola. Nunca avança sobre o oponente e nunca entra na pequena área.
class MarkOpponent : public BT::SyncActionNode {
public:
    using BT::SyncActionNode::SyncActionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

/// Estágio 2.0 (defensor ativo): quando a bola está perto do zagueiro e fora da pequena área,
/// posiciona-se atrás dela fora da área para dar um passe a um aliado livre ou afastar para
/// frente/fora (chute <= 3,0 m/s). Retorna SUCCESS quando pronto para o KickBall.
class DefenderClear : public BT::StatefulActionNode {
public:
    using BT::StatefulActionNode::StatefulActionNode;
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    struct KickPlan {
        double dir_x{1.0};
        double dir_y{0.0};
        double speed{3.0};
        int pass_id{-1};
    };

    KickPlan planKick(
        double ball_x, double ball_y, double my_goal_x,
        double robot_x, double robot_y,
        double field_length, double field_width,
        uint32_t self_id, uint32_t gk_id,
        const oxebots_interfaces::msg::GameData::SharedPtr& game_data);

    double last_target_x_{-99999.0};
    double last_target_y_{-99999.0};
    double last_target_w_{-99999.0};
    uint8_t last_planner_type_{oxebots_interfaces::msg::RobotGoal::PLANNER_DSTAR};
    double last_goal_publish_s_{-99999.0};

    int last_pass_id_{-1};
    double last_clear_angle_{std::numeric_limits<double>::quiet_NaN()};
    bool plan_locked_{false};
    KickPlan locked_plan_{};

    bool contact_since_valid_{false};
    double contact_since_s_{0.0};
};

/// Estágio 2.2 (legado/utilitário): Variante de GoToPoint com clamp de pequena área.
/// Retorna RUNNING indefinidamente — a terminação vem do KickBall no Parallel.
/// Republica o goal a cada tick para acompanhar alvos móveis (ex.: bola solta).
class GoToClamped : public BT::StatefulActionNode {
public:
    using BT::StatefulActionNode::StatefulActionNode;
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override {}
    static BT::PortsList providedPorts();
private:
    BT::NodeStatus publishClamped();
};
