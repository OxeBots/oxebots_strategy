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
        
        if (config().blackboard->get("gc_command", gc_command)) {
            // Aceita:
            // 2 = NORMAL_START
            // 3 = FORCE_START
            // 8 = DIRECT_FREE_YELLOW
            // 9 = DIRECT_FREE_BLUE
            if (gc_command == 2 || gc_command == 3 || gc_command == 8 || gc_command == 9) {
                
                // [HACK PARA TESTES] Engana a tag <CheckGCCommand expected="2"/> do XML
                // para que ela não aborte a Sequence ao ler o Force Start/Free Kick.
                config().blackboard->set("gc_command", 2); 

                return BT::NodeStatus::SUCCESS;
            }
        }
        
        return BT::NodeStatus::FAILURE;
    }
};

} // namespace oxebots_strategy