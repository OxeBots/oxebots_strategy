#pragma once
#include <behaviortree_cpp/condition_node.h>
#include <string>

namespace oxebots_strategy {

class IsNormalFreeKick : public BT::ConditionNode
{
public:
    IsNormalFreeKick(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {}; 
    }

    BT::NodeStatus tick() override
    {
        int gc_command;
        
        // Lê a variável "gc_command" alimentada pelo seu referee_callback no Blackboard
        if (config().blackboard->get("gc_command", gc_command)) {
            // Comando 2 corresponde a NORMAL_START no protobuf da SSL
            if (gc_command == 2) {
                return BT::NodeStatus::SUCCESS;
            }
        }
        
        // Mantém a árvore travada caso o jogo esteja parado
        return BT::NodeStatus::FAILURE;
    }
};

} // namespace oxebots_strategy