#pragma once

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include <cstdint>

// ---------------------------------------------------------------------------
// Tipos compartilhados entre os nós defensivos
// ---------------------------------------------------------------------------
struct Pose2D   { double x, y, theta; };
struct Vector2D { double x, y; };

// ---------------------------------------------------------------------------
// Infraestrutura de controle (publisher singleton para /robot_goal)
// ---------------------------------------------------------------------------
void configureDefenderController(const rclcpp::Node::SharedPtr& node, uint32_t robot_id);
void sendToController(const Pose2D& target);

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

/// Estágios 1 e 2.2: posiciona o defensor na linha bola -> centro do gol, a um raio fora da
/// pequena área (cobre o ângulo do gol como o PositionBisector do goleiro). Nunca cruza o
/// meio-campo e nunca entra na pequena área.
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

/// Estágio 2.2: Variante de GoToPoint com clamp de pequena área.
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