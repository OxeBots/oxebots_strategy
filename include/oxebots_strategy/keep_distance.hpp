#ifndef KEEP_DISTANCE_HPP
#define KEEP_DISTANCE_HPP

#include "behaviortree_cpp/action_node.h"

namespace oxebots_strategy
{

class KeepDistance : public BT::SyncActionNode
{
public:
    KeepDistance(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

} // namespace oxebots_strategy

#endif // KEEP_DISTANCE_HPP