#ifndef HAS_LINE_OF_SIGHT_HPP
#define HAS_LINE_OF_SIGHT_HPP

#include "behaviortree_cpp/condition_node.h"

namespace oxebots_strategy
{

class HasLineOfSight : public BT::ConditionNode
{
public:
    HasLineOfSight(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

} // namespace oxebots_strategy

#endif // HAS_LINE_OF_SIGHT_HPP