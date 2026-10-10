#ifndef AIM_AT_HPP
#define AIM_AT_HPP

#include "behaviortree_cpp/action_node.h"

namespace oxebots_strategy
{

class AimAt : public BT::SyncActionNode
{
public:
    AimAt(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

} // namespace oxebots_strategy

#endif // AIM_AT_HPP