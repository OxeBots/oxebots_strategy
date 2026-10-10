#include "oxebots_strategy/calculate_kick_off_pose.hpp"

namespace oxebots_strategy
{

CalculateKickOffPose::CalculateKickOffPose(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList CalculateKickOffPose::providedPorts()
{
    return {
        BT::InputPort<unsigned int>("robot_id", "ID do robô para calcular o espaçamento individual"),
        BT::OutputPort<double>("target_x", "Coordenada X alvo no campo de defesa"),
        BT::OutputPort<double>("target_y", "Coordenada Y alvo distribuída")
    };
}

BT::NodeStatus CalculateKickOffPose::tick()
{
    auto robot_id_res = getInput<unsigned int>("robot_id");
    if (!robot_id_res)
    {
        // Falha se o ID não foi passado na árvore
        return BT::NodeStatus::FAILURE; 
    }

    int id = robot_id_res.value();

    // Regra 5.3.2: Defensores devem estar no próprio campo (X negativo)
    // O raio do círculo central é 500mm, então -1000.0 garante recuo seguro.
    double target_x = -1000.0;

    // Espalha os robôs verticalmente baseando-se no ID (ex: 400mm de distância entre cada)
    // Subtrai um offset para que o time não fique concentrado apenas em um lado do eixo Y
    double target_y = (id * 400.0) - 1200.0; 

    // O goleiro (ID 0) tem sua própria árvore, mas por segurança, se ele cair aqui:
    if (id == 0) {
        target_x = -4000.0; // Perto do próprio gol
        target_y = 0.0;
    }

    setOutput("target_x", target_x);
    setOutput("target_y", target_y);

    return BT::NodeStatus::SUCCESS;
}

} // namespace oxebots_strategy