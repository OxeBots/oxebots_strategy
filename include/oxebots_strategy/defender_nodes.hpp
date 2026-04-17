#pragma once

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include <cstdint>


struct Pose2D {
    double x, y, theta;
};

struct Vector2D {
    double x, y;
};

void configureDefenderController(const rclcpp::Node::SharedPtr& node, uint32_t robot_id);
void sendToController(const Pose2D& target);

// Conditions
class IsBallInOpponentField : public BT::ConditionNode {
public:
    using BT::ConditionNode::ConditionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class IsOpponentControllingBall : public BT::ConditionNode {
public:
    using BT::ConditionNode::ConditionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class IsBallNearGoal : public BT::ConditionNode {
public:
    using BT::ConditionNode::ConditionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class IsDefenderCloserThanAttacker : public BT::ConditionNode {
public:
    using BT::ConditionNode::ConditionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class IsBallMovingTowardsGoal : public BT::ConditionNode {
public:
    using BT::ConditionNode::ConditionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

// Actions
class GoToSafe : public BT::SyncActionNode {
public:
    using BT::SyncActionNode::SyncActionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class BlockShot : public BT::SyncActionNode {
public:
    using BT::SyncActionNode::SyncActionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class PressureBall : public BT::SyncActionNode {
public:
    using BT::SyncActionNode::SyncActionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class BlockAngle : public BT::SyncActionNode {
public:
    using BT::SyncActionNode::SyncActionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class InterceptBall : public BT::SyncActionNode {
public:
    using BT::SyncActionNode::SyncActionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class DefensivePosition : public BT::SyncActionNode {
public:
    using BT::SyncActionNode::SyncActionNode;
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};