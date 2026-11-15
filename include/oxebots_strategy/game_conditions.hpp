#pragma once

#include "behaviortree_cpp/condition_node.h"

#include <string>

namespace oxebots_strategy
{

class CheckGameStateCondition : public BT::ConditionNode
{
public:
  CheckGameStateCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class WeAreFavoredCondition : public BT::ConditionNode
{
public:
  WeAreFavoredCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class IsDataFreshCondition : public BT::ConditionNode
{
public:
  IsDataFreshCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class HasRoleCondition : public BT::ConditionNode
{
public:
  HasRoleCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class IsCornerKickCondition : public BT::ConditionNode
{
public:
  IsCornerKickCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class IsThrowInCondition : public BT::ConditionNode
{
public:
  IsThrowInCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace oxebots_strategy
