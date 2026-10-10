#include "oxebots_strategy/has_line_of_sight.hpp"

namespace oxebots_strategy
{

HasLineOfSight::HasLineOfSight(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList HasLineOfSight::providedPorts()
{
    return {
        BT::InputPort<unsigned int>("robot_id", "ID do robô"),
        BT::InputPort<double>("target_x", "Alvo X"),
        BT::InputPort<double>("target_y", "Alvo Y")
    };
}

BT::NodeStatus HasLineOfSight::tick()
{
    // OBSERVAÇÃO: A implementação real disso exigiria checar o mapa do D* ou os dados do filtro de Kalman.
    // Para resolver o erro de compilação da árvore agora, deixamos um stub que sempre retorna SUCCESS (caminho limpo).
    // Vocês precisarão conectar isso ao sistema de visão depois.
    return BT::NodeStatus::SUCCESS; 
}

} // namespace oxebots_strategy