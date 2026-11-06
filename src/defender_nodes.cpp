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

// Lê um parâmetro geométrico estritamente positivo: da porta `port`, senão da blackboard raiz
// (`key`). Sem ele (ausente ou <= 0) devolve false e o nó deve FALHAR: sem a geometria da área o
// defensor não sabe onde fica a área do goleiro e entraria nela (R-22, SC-17).
bool readPositiveGeometry(const BT::TreeNode & node, const char * port, const char * key, double & value)
{
    value = 0.0;
    if (!node.getInput(port, value)) {
        auto root_bb = node.config().blackboard ? node.config().blackboard->rootBlackboard() : nullptr;
        if (!root_bb || !root_bb->get(key, value)) {
            value = 0.0;
        }
    }
    if (value > 0.0) {
        return true;
    }
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("defender_nodes"), clock, 2000,
        "%s: %s ausente ou invalido (%.1f); sem ele o defensor nao sabe onde fica a area do "
        "goleiro. Falhando em vez de supor um valor.", node.name().c_str(), key, value);
    return false;
}

bool readPenaltyDepth(const BT::TreeNode & node, double & depth)
{
    return readPositiveGeometry(node, "penalty_area_depth", "penalty_area_depth", depth);
}

// Distância (mm) do centro do gol até a borda da pequena área (inflada pela margem do robô) ao
// longo da direção unitária `u` (apontando para dentro do campo).
double areaExitDistance(const Vector2D & u, double depth, double width)
{
    constexpr double kEps = 1e-6;
    const double reach_x = (depth + kDefenderAreaMarginMm) / std::max(std::abs(u.x), kEps);
    const double reach_y = (width * 0.5 + kDefenderAreaMarginMm) / std::max(std::abs(u.y), kEps);
    return std::min(reach_x, reach_y);
}

// Posição do defensor na linha bola -> centro do gol, a um raio R do centro do gol (como o
// PositionBisector do goleiro, mas fora da área). R cresce com a distância da bola, nunca fica
// dentro da pequena área e nunca passa do meio-campo. Com a bola colada à área, fecha o ângulo
// pelo mesmo lado da bola (nunca troca de lado).
struct GoalArcParams
{
    double ratio;         // R = ratio * distância bola-gol
    double standoff;      // folga mínima fora da área (mm)
    double max_radius;    // raio máximo (mm)
    double depth;
    double width;
    double y_limit;       // |y| máximo do alvo (0 = sem limite)
};

// Folga mínima entre o defensor e a bola. Abaixo disso o defensor já está em cima da bola (a bola está
// dentro do anel de segurança da área) e o ângulo é fechado pelo lado. Com 650 mm isso valia para quase
// toda a nossa metade (o anel já tem ~1 m) e o defensor saía da linha bola-gol com a bola "perto".
constexpr double kAttackerClearanceMm = 350.0;
constexpr double kMidfieldMarginMm    = 180.0;
constexpr double kMaxArcAngleRad      = 1.2;   // ~69 graus em relação ao eixo do campo
constexpr double kReboundShiftRad     = 0.6;   // ~34 graus

Pose2D goalArcTarget(const Pose2D & ball, double my_goal_x, const GoalArcParams & prm)
{
    const double inward = (my_goal_x < 0.0) ? 1.0 : -1.0;   // sentido +x ou -x para dentro do campo

    // Ângulo da linha gol->bola em relação ao eixo do campo (bola atrás da linha de fundo = 0).
    const double along = std::max((ball.x - my_goal_x) * inward, 0.0);
    double angle = std::atan2(ball.y - kGoalCenterY, along);
    const double dist = std::hypot(along, ball.y - kGoalCenterY);
    angle = std::clamp(angle, -kMaxArcAngleRad, kMaxArcAngleRad);

    auto place = [&](double a) {
        const Vector2D u{inward * std::cos(a), std::sin(a)};
        const double r_exit = areaExitDistance(u, prm.depth, prm.width);
        double r = std::clamp(prm.ratio * dist, r_exit + prm.standoff,
                              std::max(r_exit + prm.standoff, prm.max_radius));
        // Entre a bola e o gol: nunca além de (distância da bola - folga). Com a bola perto da área o
        // defensor encosta na borda da área (r_exit) em vez de parar ALÉM da bola, olhando para ela.
        r = std::min(r, std::max(r_exit, dist - kAttackerClearanceMm));
        // Meio-campo: o raio não pode empurrar o defensor para além de (meia-extensão - margem).
        const double mid_cap = (std::abs(my_goal_x) - kMidfieldMarginMm) / std::max(std::abs(u.x), 1e-6);
        r = std::max(r_exit, std::min(r, mid_cap));
        Pose2D t{my_goal_x + u.x * r, kGoalCenterY + u.y * r, 0.0};
        if (prm.y_limit > 0.0) {
            t.y = std::clamp(t.y, -prm.y_limit, prm.y_limit);
        }
        return t;
    };

    Pose2D target = place(angle);
    const double gap = std::hypot(target.x - ball.x, target.y - ball.y);
    if (gap < kAttackerClearanceMm) {
        // Bola dentro do anel de segurança da área: entre ela e o gol não cabe o defensor, e o ponto da
        // reta ficaria à frente da bola, no caminho do atacante. Aproxima o raio do eixo do gol, do
        // MESMO lado da bola e sem cruzá-lo (nunca troca de lado), tanto mais quanto mais perto ela
        // estiver; o deslocamento cresce de forma contínua a partir de kAttackerClearanceMm.
        const double side  = (angle >= 0.0) ? 1.0 : -1.0;
        const double shift = kReboundShiftRad * (1.0 - gap / kAttackerClearanceMm);
        target = place(side * std::max(std::abs(angle) - shift, 0.0));
    }
    return target;
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
        BT::InputPort<double>("ball_x", "X da bola (mm)"),
        BT::InputPort<double>("my_goal_x", "{my_goal_x}",
            "X do proprio gol (mm), define o lado do campo. Padrao: {my_goal_x} da blackboard"),
        BT::InputPort<bool>("is_yellow",
            "Opcional (legado): so vale se my_goal_x nao estiver disponivel; assume que o amarelo "
            "ataca o x negativo, o que deixa de ser verdade na troca de lados")
    };
}

BT::NodeStatus IsBallInOpponentField::tick()
{
    double ball_x = 0.0;
    if (!getInput("ball_x", ball_x)) {
        return BT::NodeStatus::FAILURE;
    }

    // Lado pelo gol proprio (vem do GameState/geometria): a bola esta no campo adversario quando
    // fica do lado OPOSTO ao nosso gol. Nao depende da cor, entao vale depois da troca de lados.
    double my_goal_x = 0.0;
    if (getInput("my_goal_x", my_goal_x) && my_goal_x != 0.0) {
        return (ball_x * my_goal_x < 0.0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    bool is_yellow = false;
    if (getInput("is_yellow", is_yellow)) {
        return (is_yellow ? (ball_x < 0.0) : (ball_x > 0.0)) ? BT::NodeStatus::SUCCESS
                                                             : BT::NodeStatus::FAILURE;
    }

    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("defender_nodes"), clock, 2000,
        "IsBallInOpponentField: sem my_goal_x (e sem is_yellow) nao da para saber o lado do campo.");
    return BT::NodeStatus::FAILURE;
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
// Estágios 1 e 2.2 — cobertura do gol com a bola longe do nosso atacante/oponente.
// Posiciona o defensor na linha bola -> centro do gol, fora da pequena área, como o goleiro faz
// com o PositionBisector: o ângulo bola-gol decide onde ele fica, não o y da bola. Nunca cruza o
// meio-campo e nunca entra na pequena área.
// ---------------------------------------------------------------------------

BT::PortsList ShadowBall::providedPorts()
{
    return {
        BT::InputPort<Pose2D>("ball"),
        BT::InputPort<Vector2D>("ball_vel"),
        BT::InputPort<double>("my_goal_x"),
        BT::InputPort<double>("ratio", 0.5,
            "Raio do defensor como fração da distância bola-gol (limitado por standoff/max_radius)"),
        BT::InputPort<double>("standoff", 300.0,
            "Folga mínima (mm) entre a borda da área (já com a margem do robô) e o defensor"),
        BT::InputPort<double>("max_radius", 2500.0,
            "Raio máximo (mm) do defensor a partir do centro do gol"),
        BT::InputPort<double>("predict_time", 0.3,
            "Horizonte de predição da trajetória da bola (s)"),
        BT::InputPort<double>("penalty_area_depth",
            "Profundidade da pequena área a partir da linha de fundo (mm)"),
        BT::InputPort<double>("penalty_area_width",
            "Largura da pequena área (mm); sem a porta vale {penalty_area_width} da blackboard")
    };
}

BT::NodeStatus ShadowBall::tick()
{
    Pose2D   ball{};
    Vector2D vel{};
    double my_goal_x     = 0.0;
    double predict_time  = 0.3;
    GoalArcParams prm{0.5, 300.0, 2500.0, 0.0, 0.0, 0.0};

    if (!getInput("ball", ball) || !getInput("ball_vel", vel) ||
        !getInput("my_goal_x", my_goal_x)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("ratio",        prm.ratio);
    getInput("standoff",     prm.standoff);
    getInput("max_radius",   prm.max_radius);
    getInput("predict_time", predict_time);
    if (!readPenaltyDepth(*this, prm.depth) ||
        !readPositiveGeometry(*this, "penalty_area_width", "penalty_area_width", prm.width)) {
        return BT::NodeStatus::FAILURE;
    }

    // Largura do campo (opcional): o defensor fica a 300 mm da linha lateral.
    if (auto root_bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr) {
        double field_width = 0.0;
        if (root_bb->get("field_width", field_width) && field_width > 0.0) {
            // Nunca menor que a meia-largura da área mais a margem, ou o clamp empurraria o alvo para dentro dela.
            prm.y_limit = std::max(field_width * 0.5 - 300.0, prm.width * 0.5 + kDefenderAreaMarginMm);
        }
    }

    // Bola prevista: só quando ela vem EM DIREÇÃO ao nosso gol (chute adversário), para o defensor
    // já ficar na trajetória. Bola parada, lenta ou saindo (nosso chute para o campo adversário) usa a
    // posição atual; senão o defensor "perseguia" a bola chutada para fora e saía da linha bola-gol.
    constexpr double kMinApproachSpeedMmS = 500.0;
    constexpr double kMaxLeadMm = 1200.0;
    const double inward_dir = (my_goal_x < 0.0) ? 1.0 : -1.0;
    const double toward_goal = -vel.x * inward_dir;   // > 0: indo para o nosso gol
    Pose2D pred_ball = ball;
    if (toward_goal > kMinApproachSpeedMmS) {
        const double lead_x = std::clamp(vel.x * predict_time, -kMaxLeadMm, kMaxLeadMm);
        const double lead_y = std::clamp(vel.y * predict_time, -kMaxLeadMm, kMaxLeadMm);
        pred_ball = Pose2D{ball.x + lead_x, ball.y + lead_y, 0.0};
    }

    Pose2D target = goalArcTarget(pred_ball, my_goal_x, prm);
    target.theta  = computeHeading(target, ball);

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
        BT::InputPort<double>("penalty_area_width",
            "Largura da pequena área (mm); sem a porta vale {penalty_area_width} da blackboard"),
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
    if (!readPenaltyDepth(*this, penalty_depth)) {
        return BT::NodeStatus::FAILURE;
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

    // Alvo na linha bola->gol a block_distance da bola; se a área o empurra, usa o ângulo do ShadowBall,
    // que fecha o ângulo do gol pelo MESMO lado da bola (antes pulava para o lado oposto e abria o gol).
    // Só recorre ao posicionamento por ângulo quando o clamp da área mudou o alvo, ou seja, a bola está
    // tão perto da área que os 400 mm "entre a bola e o gol" cairiam dentro dela. Antes o teste era
    // "alvo a menos de 500 mm da bola", que com block_distance=400 valia SEMPRE: o defensor nunca ficava
    // a 400 mm da bola na linha, e sim deslocado para o lado.
    const double unclamped_x = ball.x + ux * block_dist;
    if (std::abs(target_x - unclamped_x) > 1.0 ||
        std::hypot(target_x - ball.x, target_y - ball.y) < 250.0) {
        GoalArcParams prm{0.5, 300.0, 2500.0, penalty_depth, 0.0, 0.0};
        if (!readPositiveGeometry(*this, "penalty_area_width", "penalty_area_width", prm.width)) {
            return BT::NodeStatus::FAILURE;
        }
        Pose2D arc = goalArcTarget(ball, my_goal_x, prm);
        arc.theta  = computeHeading(arc, ball);
        sendToController(arc);
        return BT::NodeStatus::SUCCESS;
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
    if (!readPenaltyDepth(*this, penalty_depth)) {
        return BT::NodeStatus::FAILURE;
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
