#include "oxebots_strategy/defender_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

constexpr double kMillimetersToMeters = 0.001;
constexpr double kGoalCenterY         = 0.0;

// Ângulo da posição `from` em direção a `to`.
double computeHeading(const Pose2D& from, const Pose2D& to)
{
    return std::atan2(to.y - from.y, to.x - from.x);
}

// Clampeia 'x' para manter o robô fora da pequena área (com margem de segurança de 200mm para o raio do robô).
// Fórmula do limite: my_goal_x - copysign(depth + kDefenderAreaMarginMm, my_goal_x)
constexpr double kDefenderAreaMarginMm = 200.0;

double penaltyClamp(double x, double my_goal_x, double depth)
{
    const double limit = my_goal_x - std::copysign(depth + kDefenderAreaMarginMm, my_goal_x);
    return (my_goal_x < 0.0) ? std::max(x, limit) : std::min(x, limit);
}

// Contexto do publisher singleton para /robot_goal.
struct DefenderControllerContext
{
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub;
    uint32_t robot_id{1};
};

DefenderControllerContext& getControllerContext()
{
    static DefenderControllerContext context;
    return context;
}

}  // namespace

// ---------------------------------------------------------------------------
// Infraestrutura de controle
// ---------------------------------------------------------------------------

void configureDefenderController(const rclcpp::Node::SharedPtr& node, uint32_t robot_id)
{
    auto& context  = getControllerContext();
    auto goal_qos  = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    context.node     = node;
    context.goal_pub = node->create_publisher<oxebots_interfaces::msg::RobotGoal>("/robot_goal", goal_qos);
    context.robot_id = robot_id;
}

void sendToController(const Pose2D& target)
{
    auto& context = getControllerContext();
    if (!context.goal_pub || !context.node) return;

    auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
    goal_msg->robot_id                      = context.robot_id;
    goal_msg->pose.header.stamp             = context.node->now();
    goal_msg->pose.header.frame_id          = "map";
    goal_msg->pose.pose.position.x          = target.x * kMillimetersToMeters;
    goal_msg->pose.pose.position.y          = target.y * kMillimetersToMeters;
    goal_msg->pose.pose.orientation.x       = 0.0;
    goal_msg->pose.pose.orientation.y       = 0.0;
    goal_msg->pose.pose.orientation.z       = std::sin(target.theta * 0.5);
    goal_msg->pose.pose.orientation.w       = std::cos(target.theta * 0.5);

    context.goal_pub->publish(std::move(goal_msg));
}

// ---------------------------------------------------------------------------
// Condições
// ---------------------------------------------------------------------------

BT::PortsList IsBallInOpponentField::providedPorts()
{
    return {
        BT::InputPort<double>("ball_x"),
        BT::InputPort<bool>("is_yellow")
    };
}

BT::NodeStatus IsBallInOpponentField::tick()
{
    double ball_x  = 0.0;
    bool is_yellow = false;

    if (!getInput("ball_x", ball_x) || !getInput("is_yellow", is_yellow)) {
        return BT::NodeStatus::FAILURE;
    }

    const bool in_opponent_field = is_yellow ? (ball_x < 0.0) : (ball_x > 0.0);
    return in_opponent_field ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsOpponentControllingBall::providedPorts()
{
    return { BT::InputPort<bool>("opponent_controlling") };
}

BT::NodeStatus IsOpponentControllingBall::tick()
{
    bool opponent_controlling = false;
    if (!getInput("opponent_controlling", opponent_controlling)) {
        return BT::NodeStatus::FAILURE;
    }
    return opponent_controlling ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// Ação: ShadowBall
// Estágio 1 — bola no campo adversário.
// Posiciona o defensor a distância fixa da bola prevista, sem cruzar o
// meio-campo e sem entrar na pequena área.
// ---------------------------------------------------------------------------

BT::PortsList ShadowBall::providedPorts()
{
    return {
        BT::InputPort<Pose2D>("ball"),
        BT::InputPort<Vector2D>("ball_vel"),
        BT::InputPort<double>("my_goal_x"),
        BT::InputPort<double>("follow_distance", 1200.0,
            "Distância fixa atrás da bola em direção ao próprio gol (mm)"),
        BT::InputPort<double>("predict_time", 0.3,
            "Horizonte de predição da trajetória da bola (s)"),
        BT::InputPort<double>("penalty_area_depth",
            "Profundidade da pequena área a partir da linha de fundo (mm)"),
        BT::InputPort<double>("y_limit", 900.0,
            "Limite lateral do defensor em Y (mm)")
    };
}

BT::NodeStatus ShadowBall::tick()
{
    Pose2D   ball{};
    Vector2D vel{};
    double my_goal_x     = 0.0;
    double follow_dist   = 1200.0;
    double predict_time  = 0.3;
    double penalty_depth = 0.0;
    double y_limit       = 900.0;

    if (!getInput("ball", ball) || !getInput("ball_vel", vel) ||
        !getInput("my_goal_x", my_goal_x)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("follow_distance", follow_dist);
    getInput("predict_time",    predict_time);
    auto root_bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
    if (!getInput("penalty_area_depth", penalty_depth) && root_bb) {
        (void)root_bb->get("penalty_area_depth", penalty_depth);
    }
    getInput("y_limit", y_limit);

    // 1. Prever posição futura da bola
    const double pred_x = ball.x + vel.x * predict_time;
    const double pred_y = ball.y + vel.y * predict_time;

    // 2. Recuar follow_dist em direção ao próprio gol a partir da bola prevista
    //    sign: -1 para azul (gol em x negativo), +1 para amarelo
    const double sign = (my_goal_x < 0.0) ? -1.0 : 1.0;
    double target_x   = pred_x + sign * follow_dist;

    // 3. Borda interna da pequena área calculada a partir das dimensões dinâmicas (com margem do robô)
    const double penalty_limit = my_goal_x - std::copysign(penalty_depth + kDefenderAreaMarginMm, my_goal_x);

    // 4. Clamp: [pequena_área, meio-campo=-180 (para considerar uma margem de erro)]
    if (my_goal_x < 0.0) {
        target_x = std::clamp(target_x, penalty_limit, -180.0);
    } else {
        target_x = std::clamp(target_x, 0.0, penalty_limit);
    }

    double target_y = std::clamp(pred_y, -y_limit, y_limit);

    // 5. Se a bola estiver próxima da nossa área (onde target_x bate no penalty_limit e o atacante
    // precisa contornar a bola/área), desloca o defensor lateralmente para o lado oposto da bola
    // para cobrir o rebote sem bloquear a rota do atacante nem travar os dois robôs juntos.
    if (std::abs(pred_x - my_goal_x) < (penalty_depth + kDefenderAreaMarginMm + 700.0) &&
        std::hypot(target_x - pred_x, target_y - pred_y) < 650.0) {
        double shift_dir = (pred_y >= 0.0) ? -1.0 : 1.0;
        target_y = std::clamp(pred_y + shift_dir * 700.0, -y_limit, y_limit);
    }

    Pose2D target{};
    target.x     = target_x;
    target.y     = target_y;
    target.theta = computeHeading(target, ball);

    sendToController(target);
    return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// Ação: MarkOpponent
// Estágio 2.1 — oponente com posse de bola.
// Posiciona o defensor na linha bola→gol a distância fixa da bola.
// Nunca avança sobre o oponente e nunca entra na pequena área.
// ---------------------------------------------------------------------------

BT::PortsList MarkOpponent::providedPorts()
{
    return {
        BT::InputPort<Pose2D>("ball"),
        BT::InputPort<double>("my_goal_x"),
        BT::InputPort<double>("block_distance", 400.0,
            "Distância mínima do oponente ao longo da linha bola→gol (mm)"),
        BT::InputPort<double>("penalty_area_depth",
            "Profundidade da pequena área a partir da linha de fundo (mm)"),
        BT::InputPort<double>("y_limit", 1200.0,
            "Limite lateral do defensor em Y (mm)")
    };
}

BT::NodeStatus MarkOpponent::tick()
{
    Pose2D ball{};
    double my_goal_x     = 0.0;
    double block_dist    = 400.0;
    double penalty_depth = 0.0;
    double y_limit       = 1200.0;

    if (!getInput("ball", ball) || !getInput("my_goal_x", my_goal_x)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("block_distance", block_dist);
    auto root_bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
    if (!getInput("penalty_area_depth", penalty_depth) && root_bb) {
        (void)root_bb->get("penalty_area_depth", penalty_depth);
    }
    getInput("y_limit", y_limit);

    // 1. Vetor unitário da bola em direção ao centro do próprio gol
    const double dx   = my_goal_x - ball.x;
    const double dy   = kGoalCenterY - ball.y;
    const double dist = std::hypot(dx, dy);

    if (dist < 1.0) {
        // Bola praticamente dentro do gol — posicionar na borda da pequena área
        Pose2D target{};
        target.x     = my_goal_x - std::copysign(penalty_depth + kDefenderAreaMarginMm, my_goal_x);
        target.y     = 0.0;
        target.theta = computeHeading(target, ball);
        sendToController(target);
        return BT::NodeStatus::SUCCESS;
    }

    const double ux = dx / dist;
    const double uy = dy / dist;

    // 2+3+4. Alvo com clamp de pequena área
    const double target_x = penaltyClamp(ball.x + ux * block_dist, my_goal_x, penalty_depth);
    double target_y = std::clamp(ball.y + uy * block_dist, -y_limit, y_limit);

    // Se o clamp da área empurrou o defensor para muito perto da bola (onde o atacante já vai disputar),
    // abre lateralmente para cobrir o outro lado do gol sem travar o corredor do atacante.
    if (std::hypot(target_x - ball.x, target_y - ball.y) < 500.0) {
        double shift_dir = (ball.y >= 0.0) ? -1.0 : 1.0;
        target_y = std::clamp(ball.y + shift_dir * 650.0, -y_limit, y_limit);
    }

    Pose2D target{};
    target.x     = target_x;
    target.y     = target_y;
    target.theta = computeHeading(target, ball);

    sendToController(target);
    return BT::NodeStatus::SUCCESS;
}
// ---------------------------------------------------------------------------
// Ação: GoToClamped
// Estágio 2.2 — oponente sem posse de bola.
// Equivalente ao GoToPoint padrão, mas o X alvo é clampado para nunca
// entrar na pequena área. Retorna RUNNING indefinidamente; a terminação
// é controlada externamente pelo Parallel + KickBall.
// ---------------------------------------------------------------------------

BT::PortsList GoToClamped::providedPorts()
{
    return {
        BT::InputPort<double>("x",
            "Coordenada X do alvo em mm (ex.: {ball_x})"),
        BT::InputPort<double>("y",
            "Coordenada Y do alvo em mm (ex.: {ball_y})"),
        BT::InputPort<double>("my_goal_x",
            "X do proprio gol (usado para calcular a borda da pequena area)"),
        BT::InputPort<double>("penalty_area_depth",
            "Profundidade da pequena area a partir da linha de fundo (mm)")
    };
}

BT::NodeStatus GoToClamped::publishClamped()
{
    double x             = 0.0;
    double y             = 0.0;
    double my_goal_x     = 0.0;
    double penalty_depth = 0.0;

    if (!getInput("x", x) || !getInput("y", y) || !getInput("my_goal_x", my_goal_x)) {
        return BT::NodeStatus::FAILURE;
    }
    auto root_bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
    if (!getInput("penalty_area_depth", penalty_depth) && root_bb) {
        (void)root_bb->get("penalty_area_depth", penalty_depth);
    }

    // Limita o alvo X fora da pequena área com base na geometria dinâmica
    x = penaltyClamp(x, my_goal_x, penalty_depth);

    // Orientação: apontar para o gol adversário (opponent_goal_x no Blackboard)
    double op_x = -my_goal_x, op_y = 0.0;
    if (root_bb) {
        (void)root_bb->get("opponent_goal_x", op_x);
        (void)root_bb->get("opponent_goal_y", op_y);
    }
    const double theta = std::atan2(op_y - y, op_x - x);

    sendToController(Pose2D{x, y, theta});
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToClamped::onStart()   { return publishClamped(); }
BT::NodeStatus GoToClamped::onRunning() { return publishClamped(); }
