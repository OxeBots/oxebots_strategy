#include "oxebots_strategy/keep_distance.hpp"
#include <cmath>

namespace oxebots_strategy
{

KeepDistance::KeepDistance(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList KeepDistance::providedPorts()
{
    return {
        BT::InputPort<unsigned int>("robot_id", "ID do robô"),
        BT::InputPort<double>("distance", "Distância mínima a manter (ex: 500.0)"),
        BT::InputPort<double>("ball_x", "Posição X da bola"),
        BT::InputPort<double>("ball_y", "Posição Y da bola"),
        BT::InputPort<double>("robot_x", "Posição X do robô"),
        BT::InputPort<double>("robot_y", "Posição Y do robô"),
        BT::OutputPort<double>("target_x", "Coordenada X para onde recuar"),
        BT::OutputPort<double>("target_y", "Coordenada Y para onde recuar")
    };
}

BT::NodeStatus KeepDistance::tick()
{
    auto robot_id = getInput<unsigned int>("robot_id");
    auto distance = getInput<double>("distance");
    auto ball_x = getInput<double>("ball_x");
    auto ball_y = getInput<double>("ball_y");
    auto robot_x = getInput<double>("robot_x");
    auto robot_y = getInput<double>("robot_y");

    if (!robot_id || !distance || !ball_x || !ball_y || !robot_x || !robot_y) {
        return BT::NodeStatus::FAILURE;
    }

    // Calcula o vetor da bola para o robô
    double dx = robot_x.value() - ball_x.value();
    double dy = robot_y.value() - ball_y.value();
    double current_dist = std::hypot(dx, dy);

    // Se já estiver na distância segura, fica onde está
    if (current_dist >= distance.value()) {
        setOutput("target_x", robot_x.value());
        setOutput("target_y", robot_y.value());
        return BT::NodeStatus::SUCCESS;
    }

    // Se estiver muito perto, calcula um ponto para recuar na mesma linha (vetor normalizado)
    double ratio = distance.value() / (current_dist > 0.001 ? current_dist : 0.001);
    double target_x = ball_x.value() + (dx * ratio);
    double target_y = ball_y.value() + (dy * ratio);

    setOutput("target_x", target_x);
    setOutput("target_y", target_y);

    return BT::NodeStatus::SUCCESS;
}

} // namespace oxebots_strategy