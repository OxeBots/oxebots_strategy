#include "oxebots_strategy/aim_at.hpp"
#include <cmath>

namespace oxebots_strategy
{

AimAt::AimAt(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList AimAt::providedPorts()
{
    return {
        BT::InputPort<unsigned int>("robot_id", "ID do robô"),
        BT::InputPort<double>("target_x", "Ponto X para onde mirar"),
        BT::InputPort<double>("target_y", "Ponto Y para onde mirar"),
        BT::InputPort<double>("robot_x", "Posição X atual do robô"),
        BT::InputPort<double>("robot_y", "Posição Y atual do robô"),
        BT::OutputPort<double>("target_angle", "Ângulo calculado em radianos")
    };
}

BT::NodeStatus AimAt::tick()
{
    auto target_x = getInput<double>("target_x");
    auto target_y = getInput<double>("target_y");
    auto robot_x = getInput<double>("robot_x");
    auto robot_y = getInput<double>("robot_y");

    if (!target_x || !target_y || !robot_x || !robot_y) {
        return BT::NodeStatus::FAILURE;
    }

    // Calcula o ângulo (atan2) do robô em direção ao alvo
    double angle = std::atan2(target_y.value() - robot_y.value(), target_x.value() - robot_x.value());

    setOutput("target_angle", angle);
    return BT::NodeStatus::SUCCESS;
}

} // namespace oxebots_strategy