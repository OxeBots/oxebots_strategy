#ifndef CALCULATE_KICKOFF_POSE_HPP
#define CALCULATE_KICKOFF_POSE_HPP

#include "behaviortree_cpp/action_node.h"

namespace oxebots_strategy
{

class CalculateKickOffPose : public BT::SyncActionNode
{
public:
    CalculateKickOffPose(const std::string& name, const BT::NodeConfig& config);

    // Define as portas de entrada e saída do nó
    static BT::PortsList providedPorts();

    // Execução síncrona do cálculo
    BT::NodeStatus tick() override;
};

} // namespace oxebots_strategy

#endif // CALCULATE_KICKOFF_POSE_HPP