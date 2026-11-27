#include "oxebots_strategy/defender_nodes.hpp"
#include "oxebots_strategy/goalkeeper_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

namespace
{

namespace gk = oxebots_strategy::goalkeeper;

constexpr double kMillimetersToMeters = 0.001;
constexpr double kGoalCenterY         = 0.0;
constexpr double kDefaultGoalWidthMm  = 800.0;
constexpr double kDefaultFieldLenMm   = 4500.0;
constexpr double kDefaultFieldWidMm   = 3000.0;
constexpr uint32_t kNoRobotId         = 999u;

// Ângulo da posição `from` em direção a `to`.
double computeHeading(const Pose2D& from, const Pose2D& to)
{
    return std::atan2(to.y - from.y, to.x - from.x);
}

double normalizeAngleRad(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Clampeia 'x' para manter o robô fora da pequena área (com margem de segurança de 200mm para o raio do robô).
// Fórmula do limite: my_goal_x - copysign(depth + kDefenderAreaMarginMm, my_goal_x)
constexpr double kDefenderAreaMarginMm = 200.0;

double penaltyClamp(double x, double my_goal_x, double depth)
{
    const double limit = my_goal_x - std::copysign(depth + kDefenderAreaMarginMm, my_goal_x);
    return (my_goal_x < 0.0) ? std::max(x, limit) : std::min(x, limit);
}

// Garante que (x, y) fique estritamente fora da própria área inflada (profundidade + 200mm, meia-largura + 200mm).
void clampOutsideOwnArea(double & x, double & y, double my_goal_x, double depth, double width)
{
    const double x_limit = my_goal_x - std::copysign(depth + kDefenderAreaMarginMm, my_goal_x);
    const double y_limit = width * 0.5 + kDefenderAreaMarginMm;
    const bool in_x = (my_goal_x < 0.0) ? (x < x_limit) : (x > x_limit);
    const bool in_y = std::abs(y) < y_limit;
    if (!in_x || !in_y) {
        return;
    }
    const double pen_x = (my_goal_x < 0.0) ? (x_limit - x) : (x - x_limit);
    const double pen_y = y_limit - std::abs(y);
    if (pen_y < pen_x) {
        y = (y >= 0.0) ? y_limit : -y_limit;
    } else {
        x = x_limit;
    }
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

// Posição do defensor na linha bola -> referência do gol (centro por padrão, ou canto aberto na
// divisão de cobertura com o goleiro), a um raio R do gol, fora da pequena área e sem cruzar o meio-campo.
struct GoalArcParams
{
    double ratio;         // R = ratio * distância bola-gol
    double standoff;      // folga mínima fora da área (mm)
    double max_radius;    // raio máximo (mm)
    double depth;
    double width;
    double y_limit;       // |y| máximo do alvo (0 = sem limite)
    double goal_anchor_y{kGoalCenterY};
};

// Folga mínima entre o defensor e a bola. Abaixo disso o defensor já está em cima da bola (a bola está
// dentro do anel de segurança da área) e o ângulo é fechado pelo lado.
constexpr double kAttackerClearanceMm = 350.0;
constexpr double kMidfieldMarginMm    = 180.0;
constexpr double kMaxArcAngleRad      = 1.2;   // ~69 graus em relação ao eixo do campo
constexpr double kReboundShiftRad     = 0.6;   // ~34 graus

Pose2D goalArcTarget(const Pose2D & ball, double my_goal_x, const GoalArcParams & prm)
{
    const double inward = (my_goal_x < 0.0) ? 1.0 : -1.0;   // sentido +x ou -x para dentro do campo
    const double anchor_y = prm.goal_anchor_y;

    // Ângulo da linha gol(anchor_y)->bola em relação ao eixo do campo (bola atrás da linha de fundo = 0).
    const double along = std::max((ball.x - my_goal_x) * inward, 0.0);
    double angle = std::atan2(ball.y - anchor_y, along);
    const double dist = std::hypot(along, ball.y - anchor_y);
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
        Pose2D t{my_goal_x + u.x * r, anchor_y + u.y * r, 0.0};
        if (prm.y_limit > 0.0) {
            t.y = std::clamp(t.y, -prm.y_limit, prm.y_limit);
        }
        clampOutsideOwnArea(t.x, t.y, my_goal_x, prm.depth, prm.width);
        return t;
    };

    Pose2D target = place(angle);
    const double gap = std::hypot(target.x - ball.x, target.y - ball.y);
    if (gap < kAttackerClearanceMm) {
        // Bola dentro do anel de segurança da área: aproxima o raio do eixo do gol, do MESMO lado da bola.
        const double side  = (angle >= 0.0) ? 1.0 : -1.0;
        const double shift = kReboundShiftRad * (1.0 - gap / kAttackerClearanceMm);
        target = place(side * std::max(std::abs(angle) - shift, 0.0));
    }
    return target;
}

// Contexto do publisher singleton para /robot_goal e subscriber compartilhado de /game_data.
struct DefenderControllerContext
{
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_pub;
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub;
    std::mutex data_mutex;
    oxebots_interfaces::msg::GameData::SharedPtr last_game_data;
    uint32_t robot_id{1};
};

DefenderControllerContext& getControllerContext()
{
    static DefenderControllerContext context;
    return context;
}

oxebots_interfaces::msg::GameData::SharedPtr getLatestGameData()
{
    auto& context = getControllerContext();
    std::lock_guard<std::mutex> lock(context.data_mutex);
    return context.last_game_data;
}

// Fase 1 — Bloqueio de trajetória real de chute:
// Se a bola está viajando rápido em direção à boca do nosso gol (dentro de goal_width/2 + margem),
// projeta o defensor perpendicularmente sobre a reta da trajetória da bola (mantendo fora da área
// e dentro da nossa metade). Retorna std::nullopt se o chute não estiver direcionado ao gol.
std::optional<Pose2D> computeShotBlockTarget(
    const Pose2D & ball,
    const Vector2D & vel,
    double my_goal_x,
    double goal_width,
    const GoalArcParams & prm,
    const std::optional<Pose2D> & robot_pose)
{
    constexpr double kShotMinSpeedMmS = 700.0;
    constexpr double kGoalMouthMarginMm = 250.0;
    const double inward = (my_goal_x < 0.0) ? 1.0 : -1.0;
    const double toward_goal = -vel.x * inward;
    if (toward_goal < kShotMinSpeedMmS) {
        return std::nullopt;
    }

    // Tempo até a reta da bola cruzar a linha do nosso gol (x = my_goal_x)
    const double t_goal = (my_goal_x - ball.x) / vel.x;
    if (!std::isfinite(t_goal) || t_goal <= 0.02 || t_goal > 3.0) {
        return std::nullopt;
    }

    const double y_cross = ball.y + vel.y * t_goal;
    const double max_goal_y = goal_width * 0.5 + kGoalMouthMarginMm;
    if (std::abs(y_cross) > max_goal_y) {
        return std::nullopt;
    }

    // Segmento da trajetória do chute: ball -> (my_goal_x, y_cross)
    const double seg_dx = my_goal_x - ball.x;
    const double seg_dy = y_cross - ball.y;
    const double seg_len = std::hypot(seg_dx, seg_dy);
    if (seg_len < 50.0) {
        return std::nullopt;
    }
    const double ux = seg_dx / seg_len;
    const double uy = seg_dy / seg_len;

    // Referência para projeção: posição atual do defensor, ou borda segura da área se ausente
    const double area_front_x = my_goal_x - std::copysign(prm.depth + kDefenderAreaMarginMm + 120.0, my_goal_x);
    const double ref_x = robot_pose.has_value() ? robot_pose->x : area_front_x;
    const double ref_y = robot_pose.has_value() ? robot_pose->y : y_cross;

    // Projeção perpendicular sobre o segmento bola -> ponto de entrada no gol
    double along = (ref_x - ball.x) * ux + (ref_y - ball.y) * uy;
    // Garante que o ponto fique à frente da bola (sem recuar para trás dela) e antes da linha do gol
    along = std::clamp(along, std::min(150.0, seg_len * 0.5), seg_len);

    Pose2D block{ball.x + ux * along, ball.y + uy * along, 0.0};

    // Mantém na nossa metade (nunca cruza o meio-campo)
    const double mid_limit_x = (my_goal_x < 0.0) ? -kMidfieldMarginMm : kMidfieldMarginMm;
    if ((my_goal_x < 0.0 && block.x > mid_limit_x) || (my_goal_x > 0.0 && block.x < mid_limit_x)) {
        const double t_mid = (mid_limit_x - ball.x) / seg_dx;
        if (t_mid >= 0.0 && t_mid <= 1.0) {
            block.x = mid_limit_x;
            block.y = ball.y + seg_dy * t_mid;
        } else {
            block.x = mid_limit_x;
        }
    }

    // Se caiu dentro da pequena área inflada, encontra a interseção da reta do chute com a borda da área
    const double x_limit = my_goal_x - std::copysign(prm.depth + kDefenderAreaMarginMm, my_goal_x);
    const double y_area_limit = prm.width * 0.5 + kDefenderAreaMarginMm;
    const bool in_area_x = (my_goal_x < 0.0) ? (block.x < x_limit) : (block.x > x_limit);
    if (in_area_x && std::abs(block.y) < y_area_limit) {
        // Interseção com a linha frontal da área (x = x_limit) ao longo do chute
        if (std::abs(seg_dx) > 1e-6) {
            const double t_front = (x_limit - ball.x) / seg_dx;
            if (t_front >= 0.0 && t_front <= 1.0) {
                block.x = x_limit;
                block.y = ball.y + seg_dy * t_front;
            }
        }
        clampOutsideOwnArea(block.x, block.y, my_goal_x, prm.depth, prm.width);
    }

    if (prm.y_limit > 0.0) {
        block.y = std::clamp(block.y, -prm.y_limit, prm.y_limit);
        clampOutsideOwnArea(block.x, block.y, my_goal_x, prm.depth, prm.width);
    }

    block.theta = computeHeading(block, ball);
    return block;
}

// Fase 1 — Divisão de cobertura com o goleiro:
// Quando o goleiro real está cobrindo um lado do gol (|gk_y| > 50mm), o zagueiro desloca levemente
// sua âncora no gol para o lado oposto, fechando o canto aberto em vez de duplicar a mesma linha.
double computeCoordinatedGoalAnchorY(
    double my_goal_x,
    double penalty_depth,
    double goal_width,
    uint32_t self_id,
    uint32_t gk_id,
    const oxebots_interfaces::msg::GameData::SharedPtr & game_data)
{
    if (!game_data || gk_id == kNoRobotId || gk_id == self_id) {
        return kGoalCenterY;
    }

    for (const auto & ally : game_data->robots.allies) {
        if (ally.id != gk_id) continue;
        // Só coordena se o goleiro estiver efetivamente perto do nosso gol
        if (std::abs(static_cast<double>(ally.x) - my_goal_x) > penalty_depth + 500.0) {
            return kGoalCenterY;
        }
        const double gk_y = static_cast<double>(ally.y);
        constexpr double kGkDeadbandMm = 50.0;
        if (std::abs(gk_y) <= kGkDeadbandMm) {
            return kGoalCenterY;
        }
        const double max_offset = std::min(goal_width * 0.35, 220.0);
        return std::clamp(-0.65 * gk_y, -max_offset, max_offset);
    }

    return kGoalCenterY;
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
    {
        std::lock_guard<std::mutex> lock(context.data_mutex);
        context.last_game_data.reset();
    }
    context.game_data_sub = node->create_subscription<oxebots_interfaces::msg::GameData>(
        "/game_data", 10,
        [](const oxebots_interfaces::msg::GameData::SharedPtr msg) {
            auto& ctx = getControllerContext();
            std::lock_guard<std::mutex> lock(ctx.data_mutex);
            ctx.last_game_data = msg;
        });
}

void setDefenderGameDataForTest(const oxebots_interfaces::msg::GameData::SharedPtr& msg)
{
    auto& context = getControllerContext();
    std::lock_guard<std::mutex> lock(context.data_mutex);
    context.last_game_data = msg;
}

void sendToController(const Pose2D& target, uint8_t planner_type, bool disable_arrival_latch)
{
    auto& context = getControllerContext();
    if (!context.goal_pub || !context.node) return;

    auto goal_msg = std::make_unique<oxebots_interfaces::msg::RobotGoal>();
    goal_msg->robot_id                      = context.robot_id;
    goal_msg->planner_type                  = planner_type;
    goal_msg->disable_arrival_latch         = disable_arrival_latch;
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
// Estágios 1 e 2.2 — cobertura do gol + bloqueio direto de chutes + divisão com goleiro.
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
    GoalArcParams prm{0.5, 300.0, 2500.0, 0.0, 0.0, 0.0, kGoalCenterY};

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

    double goal_width = kDefaultGoalWidthMm;
    std::optional<Pose2D> robot_pose;
    uint32_t self_id = getControllerContext().robot_id;
    uint32_t gk_id = 0u;

    if (auto root_bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr) {
        double field_width = 0.0;
        if (root_bb->get("field_width", field_width) && field_width > 0.0) {
            prm.y_limit = std::max(field_width * 0.5 - 300.0, prm.width * 0.5 + kDefenderAreaMarginMm);
        }
        double gw = 0.0;
        if (root_bb->get("goal_width", gw) && gw > 0.0) {
            goal_width = gw;
        }
        double rx = 0.0, ry = 0.0, ryaw = 0.0;
        if (root_bb->get("robot_x", rx) && root_bb->get("robot_y", ry)) {
            (void)root_bb->get("robot_yaw", ryaw);
            robot_pose = Pose2D{rx, ry, ryaw};
        }
        (void)root_bb->get("robot_id", self_id);
        uint32_t bb_gk = kNoRobotId;
        if (root_bb->get("goalkeeper_id", bb_gk)) {
            gk_id = bb_gk;
        }
    }

    // 1. Bloqueio de trajetória real de chute: se a bola está vindo rápida na direção da boca do gol,
    //    intercepta a reta do chute diretamente em linha reta (sem latência do D*).
    if (const auto shot_block = computeShotBlockTarget(ball, vel, my_goal_x, goal_width, prm, robot_pose)) {
        sendToController(*shot_block, oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE, true);
        return BT::NodeStatus::SUCCESS;
    }

    // 2. Divisão de cobertura com o goleiro: desloca a âncora para o canto oposto ao do goleiro
    //    (só ativa quando há pose real do goleiro em /game_data).
    const auto game_data = getLatestGameData();
    prm.goal_anchor_y = computeCoordinatedGoalAnchorY(
        my_goal_x, prm.depth, goal_width, self_id, gk_id, game_data);

    // 3. Bola prevista quando vem para o nosso lado (ex.: cruzamento/rebote rápido fora do cone do gol)
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

    // Limite lateral coerente com a largura real do campo quando disponível na blackboard
    double field_y_limit = y_limit;
    if (auto root_bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr) {
        double field_width = 0.0;
        if (root_bb->get("field_width", field_width) && field_width > 0.0) {
            field_y_limit = std::min(y_limit, std::max(field_width * 0.5 - 300.0, 0.0));
        }
    }

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
    double target_y = std::clamp(ball.y + uy * block_dist, -field_y_limit, field_y_limit);

    // Se a área empurrou o alvo, recorre ao posicionamento por ângulo com os MESMOS parâmetros
    // do ShadowBall do Estágio 2.2 (ratio=0.35, y_limit consistente), evitando saltos bruscos.
    const double unclamped_x = ball.x + ux * block_dist;
    if (std::abs(target_x - unclamped_x) > 1.0 ||
        std::hypot(target_x - ball.x, target_y - ball.y) < 250.0) {
        GoalArcParams prm{0.35, 300.0, 2500.0, penalty_depth, 0.0, field_y_limit, kGoalCenterY};
        if (!readPositiveGeometry(*this, "penalty_area_width", "penalty_area_width", prm.width)) {
            return BT::NodeStatus::FAILURE;
        }
        if (prm.y_limit > 0.0) {
            prm.y_limit = std::max(prm.y_limit, prm.width * 0.5 + kDefenderAreaMarginMm);
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
// Ação: DefenderClear
// Estágio 2.0 — Zagueiro ativo perto da bola (fora da pequena área):
//   - Tenta passe limpo para um aliado de linha livre à frente/lateral;
//   - Se não houver passe seguro ou houver pressão adversária, afasta firme
//     para a frente/lateral (para fora se necessário), sempre <= 3,0 m/s.
//   - Nunca entra na pequena área (margem de 200 mm, R-22) e não rouba a bola
//     se outro aliado de linha já estiver com a posse colado nela.
// ---------------------------------------------------------------------------

BT::PortsList DefenderClear::providedPorts()
{
    return {
        BT::InputPort<uint32_t>("robot_id", "{robot_id}", "ID do robô defensor"),
        BT::InputPort<double>("ball_x", "Coordenada X da bola (mm)"),
        BT::InputPort<double>("ball_y", "Coordenada Y da bola (mm)"),
        BT::InputPort<double>("my_goal_x", "{my_goal_x}", "X do próprio gol (mm)"),
        BT::InputPort<double>("penalty_area_depth",
            "Opcional: profundidade da pequena área (mm); cai para a blackboard raiz"),
        BT::InputPort<double>("penalty_area_width",
            "Opcional: largura da pequena área (mm); cai para a blackboard raiz"),
        BT::OutputPort<double>("kick_speed", "Velocidade do chute escolhida (m/s): passe ou afastar")
    };
}

BT::NodeStatus DefenderClear::onStart()
{
    last_target_x_ = -99999.0;
    last_target_y_ = -99999.0;
    last_target_w_ = -99999.0;
    last_planner_type_ = oxebots_interfaces::msg::RobotGoal::PLANNER_DSTAR;
    last_goal_publish_s_ = -99999.0;
    last_pass_id_ = -1;
    last_clear_angle_ = std::numeric_limits<double>::quiet_NaN();
    plan_locked_ = false;
    contact_since_valid_ = false;
    return onRunning();
}

void DefenderClear::onHalted()
{
    plan_locked_ = false;
    contact_since_valid_ = false;
}

DefenderClear::KickPlan DefenderClear::planKick(
    double ball_x, double ball_y, double my_goal_x,
    double robot_x, double robot_y,
    double field_length, double field_width,
    uint32_t self_id, uint32_t gk_id,
    const oxebots_interfaces::msg::GameData::SharedPtr & game_data)
{
    constexpr double kPassMinDistMm = 350.0;
    constexpr double kPassMaxDistMm = 2800.0;
    constexpr double kPassMaxRobotBallDistMm = 550.0;
    constexpr double kOpponentPressureMm = 420.0;
    constexpr double kPassStickyScore = 0.15;
    constexpr double kClearKickSpeed = 3.0;      // R-25: teto 3,0 m/s
    constexpr double kPassMinKickSpeed = 1.2;
    constexpr double kPassArrivalSpeed = 1.0;
    constexpr double kBallFrictionAccel = 0.8;
    constexpr double kStopInsideMarginMm = 350.0;
    constexpr double kClearStickiness = 0.30;

    const double forward_sign = (my_goal_x < 0.0) ? 1.0 : -1.0;
    const double half_len = field_length * 0.5;
    const double half_wid = field_width * 0.5;
    const gk::Vec2 ball{ball_x, ball_y};

    // Direção nominal "para a frente" levemente inclinada para a lateral onde a bola já está
    const double side_bias = (ball_y >= 0.0) ? 0.25 : -0.25;
    const double forward_angle = std::atan2(side_bias, forward_sign);

    std::vector<gk::Vec2> opponents;
    std::vector<gk::Vec2> line_allies;
    double enemy_near = std::numeric_limits<double>::infinity();
    if (game_data) {
        for (const auto & enemy : game_data->robots.enemies) {
            const gk::Vec2 epos{enemy.x, enemy.y};
            opponents.push_back(epos);
            enemy_near = std::min(enemy_near, gk::distance(epos, ball));
        }
        for (const auto & ally : game_data->robots.allies) {
            if (ally.id == self_id || ally.id == gk_id) continue;
            line_allies.push_back({ally.x, ally.y});
        }
    }

    const bool under_pressure = enemy_near < kOpponentPressureMm;
    // Perto da linha de fundo própria, prioriza afastar direto em vez de tentar passe lateral arriscado
    const double dist_from_goal_line = (ball_x - my_goal_x) * forward_sign;
    const bool near_goal_line = dist_from_goal_line < 750.0;
    const double robot_ball_dist = std::hypot(ball_x - robot_x, ball_y - robot_y);

    // 1. Tenta encontrar um passe seguro estritamente para a frente quando já estamos perto da bola
    //    e atrás da direção do passe (evita virar `atras` negativo e dar volta na bola de longe).
    if (game_data && !under_pressure && !near_goal_line && robot_ball_dist <= kPassMaxRobotBallDistMm) {
        struct Candidate { int id; gk::Vec2 pos; double dist; double score; };
        std::vector<Candidate> candidates;
        const gk::PassParams pass{};
        for (const auto & ally : game_data->robots.allies) {
            if (ally.id == self_id || ally.id == gk_id) continue;
            const gk::Vec2 pos{ally.x, ally.y};
            const double dist = gk::distance(ball, pos);
            if (dist < kPassMinDistMm || dist > kPassMaxDistMm) continue;
            const double ux = (pos.x - ball.x) / dist;
            const double uy = (pos.y - ball.y) / dist;
            // Exige passe para a frente e robô já posicionado atrás da direção do passe
            if (forward_sign * ux < 0.15) continue;
            const double behind_pass = (ball_x - robot_x) * ux + (ball_y - robot_y) * uy;
            if (behind_pass < 0.0) continue;
            const double clearance = gk::passClearance(ball, pos, opponents, pass);
            if (clearance <= 0.0) continue;
            candidates.push_back({
                static_cast<int>(ally.id), pos, dist,
                gk::passScore(ball, pos, clearance, forward_sign, half_wid, kPassMaxDistMm)
            });
        }

        if (!candidates.empty()) {
            const Candidate * best = &candidates.front();
            for (const auto & c : candidates) {
                if (c.score > best->score) best = &c;
            }
            for (const auto & c : candidates) {
                if (c.id == last_pass_id_ && c.score >= best->score - kPassStickyScore) {
                    best = &c;
                    break;
                }
            }
            last_pass_id_ = best->id;
            const double ux = (best->pos.x - ball.x) / best->dist;
            const double uy = (best->pos.y - ball.y) / best->dist;
            const double dist_m = best->dist / 1000.0;
            const double speed = std::clamp(
                std::sqrt(kPassArrivalSpeed * kPassArrivalSpeed + 2.0 * kBallFrictionAccel * dist_m),
                kPassMinKickSpeed, kClearKickSpeed);
            return KickPlan{ux, uy, speed, best->id};
        }
    }

    // 2. Sem passe livre ou sob pressão: afasta firme para frente / lateral (pode chutar para fora
    //    se estiver pressionado e não houver corredor limpo no campo).
    last_pass_id_ = -1;
    gk::FanParams fan;
    fan.half_span_rad = 65.0 * M_PI / 180.0;
    // Sob pressão, permite chutar para fora com penalidade baixa; livre, prefere manter no campo
    fan.exit_penalty = under_pressure ? 0.15 : 0.70;

    gk::DestinationParams dest;
    dest.friction_mps2 = kBallFrictionAccel;

    const bool keep_in_field = !under_pressure;
    const gk::FanChoice choice = gk::chooseClearAngleWithPossession(
        ball, forward_angle, kClearKickSpeed, keep_in_field, line_allies, opponents,
        half_len, half_wid, gk::PassParams(), fan, dest,
        kStopInsideMarginMm, 1.8, last_clear_angle_, kClearStickiness);
    last_clear_angle_ = choice.angle;
    const double speed = std::clamp(under_pressure ? kClearKickSpeed : choice.speed, 1.5, kClearKickSpeed);
    return KickPlan{std::cos(choice.angle), std::sin(choice.angle), speed, -1};
}

BT::NodeStatus DefenderClear::onRunning()
{
    double ball_x = 0.0;
    double ball_y = 0.0;
    double my_goal_x = 0.0;
    if (!getInput("ball_x", ball_x) || !getInput("ball_y", ball_y) ||
        !getInput("my_goal_x", my_goal_x) || my_goal_x == 0.0) {
        return BT::NodeStatus::FAILURE;
    }

    double penalty_depth = 0.0;
    double penalty_width = 0.0;
    if (!readPenaltyDepth(*this, penalty_depth) ||
        !readPositiveGeometry(*this, "penalty_area_width", "penalty_area_width", penalty_width)) {
        return BT::NodeStatus::FAILURE;
    }

    auto & ctx = getControllerContext();
    uint32_t self_id = ctx.robot_id;
    (void)getInput("robot_id", self_id);

    auto root_bb = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
    uint32_t gk_id = 0u;
    double field_length = kDefaultFieldLenMm;
    double field_width = kDefaultFieldWidMm;
    double ball_vx = 0.0;
    double ball_vy = 0.0;
    double rx = 0.0, ry = 0.0, ryaw = 0.0;
    bool have_pose = false;

    if (root_bb) {
        uint32_t bb_gk = kNoRobotId;
        if (root_bb->get("goalkeeper_id", bb_gk)) {
            gk_id = bb_gk;
        }
        double fl = 0.0, fw = 0.0;
        if (root_bb->get("field_length", fl) && fl > 0.0) field_length = fl;
        if (root_bb->get("field_width", fw) && fw > 0.0) field_width = fw;
        (void)root_bb->get("ball_vx", ball_vx);
        (void)root_bb->get("ball_vy", ball_vy);
        if (root_bb->get("robot_x", rx) && root_bb->get("robot_y", ry) && root_bb->get("robot_yaw", ryaw)) {
            have_pose = true;
        }
    }

    const auto game_data = getLatestGameData();
    if (!have_pose && game_data) {
        for (const auto & ally : game_data->robots.allies) {
            if (ally.id == self_id) {
                rx = ally.x;
                ry = ally.y;
                ryaw = ally.orientation;
                have_pose = true;
                break;
            }
        }
    }
    if (!have_pose) {
        return BT::NodeStatus::FAILURE;
    }

    const double dist_to_ball = std::hypot(ball_x - rx, ball_y - ry);

    // Não rouba a bola do próprio atacante/aliado de linha que já está com ela colada no kicker
    if (game_data) {
        constexpr double kAllyControlDistMm = 220.0;
        constexpr double kAllyAdvantageMm = 120.0;
        for (const auto & ally : game_data->robots.allies) {
            if (ally.id == self_id || ally.id == gk_id) continue;
            const double ally_dist = std::hypot(ball_x - ally.x, ball_y - ally.y);
            if (ally_dist < kAllyControlDistMm && ally_dist + kAllyAdvantageMm < dist_to_ball) {
                plan_locked_ = false;
                contact_since_valid_ = false;
                return BT::NodeStatus::FAILURE;
            }
        }
    }

    // -----------------------------------------------------------------------
    // 1. Predição / Interceptação ativa de bola em movimento no nosso campo
    // -----------------------------------------------------------------------
    constexpr double kFieldMarginMm = 150.0;
    const double max_fx = std::max(0.0, field_length * 0.5 - kFieldMarginMm);
    const double max_fy = std::max(0.0, field_width * 0.5 - kFieldMarginMm);

    const double ball_speed = std::hypot(ball_vx, ball_vy);
    const double inward_dir = (my_goal_x < 0.0) ? 1.0 : -1.0;
    const double toward_goal = -ball_vx * inward_dir;

    // Prediz posição da bola no instante t levando em conta desaceleração (~800 mm/s^2)
    auto predictBallAt = [&](double t) -> std::pair<double, double> {
        if (ball_speed < 1e-3 || t <= 0.0) {
            return {ball_x, ball_y};
        }
        constexpr double kBallDecelMmS2 = 800.0;
        const double t_stop = ball_speed / kBallDecelMmS2;
        const double te = std::min(t, t_stop);
        const double travel = ball_speed * te - 0.5 * kBallDecelMmS2 * te * te;
        const double ux = ball_vx / ball_speed;
        const double uy = ball_vy / ball_speed;
        return {
            std::clamp(ball_x + ux * travel, -max_fx, max_fx),
            std::clamp(ball_y + uy * travel, -max_fy, max_fy)
        };
    };

    double aim_ball_x = ball_x;
    double aim_ball_y = ball_y;
    bool direct_intercept = false;

    if (ball_speed > 150.0 && dist_to_ball > 150.0) {
        constexpr double kDefenderSpeedMmS = 1400.0;
        constexpr double kMaxInterceptHorizonS = 0.75;
        // Busca o menor tempo t em que o defensor alcança a trajetória da bola
        double best_t = 0.0;
        double best_margin = std::numeric_limits<double>::infinity();
        bool found_reachable = false;
        for (int step = 1; step <= 15; ++step) {
            const double t = (kMaxInterceptHorizonS * static_cast<double>(step)) / 15.0;
            const auto [px, py] = predictBallAt(t);
            const double d_rob = std::hypot(px - rx, py - ry);
            const double deficit = d_rob - kDefenderSpeedMmS * t;
            if (deficit <= 100.0) {
                best_t = t;
                found_reachable = true;
                break;
            }
            if (deficit < best_margin) {
                best_margin = deficit;
                best_t = t;
            }
        }
        const auto [ix, iy] = predictBallAt(best_t);
        aim_ball_x = ix;
        aim_ball_y = iy;

        // Bola rápida vindo para o nosso gol/campo: corta diretamente a trajetória em linha reta
        if (ball_speed > 450.0 && toward_goal > 250.0 && (found_reachable || dist_to_ball > 220.0)) {
            direct_intercept = true;
        }
    }

    // Destrava o plano se a bola se afastou
    constexpr double kPlanUnlockDistMm = 450.0;
    if (plan_locked_ && dist_to_ball > kPlanUnlockDistMm) {
        plan_locked_ = false;
    }

    const KickPlan plan = plan_locked_
        ? locked_plan_
        : planKick(aim_ball_x, aim_ball_y, my_goal_x, rx, ry, field_length, field_width, self_id, gk_id, game_data);

    const double clear_dir_x = plan.dir_x;
    const double clear_dir_y = plan.dir_y;
    const double clear_angle = std::atan2(clear_dir_y, clear_dir_x);

    // Projeções em relação à bola REAL (para critério físico de contato/chute)
    const double real_r_to_ball_x = ball_x - rx;
    const double real_r_to_ball_y = ball_y - ry;
    const double real_behind_proj = real_r_to_ball_x * clear_dir_x + real_r_to_ball_y * clear_dir_y;
    const double angle_to_ball = std::atan2(real_r_to_ball_y, real_r_to_ball_x);

    // Projeções em relação ao ponto de interceptação (para navegação/aproximação)
    const double vec_r_to_aim_x = aim_ball_x - rx;
    const double vec_r_to_aim_y = aim_ball_y - ry;
    const double behind_proj = vec_r_to_aim_x * clear_dir_x + vec_r_to_aim_y * clear_dir_y;
    const double lateral_err = std::abs(-vec_r_to_aim_x * clear_dir_y + vec_r_to_aim_y * clear_dir_x);

    const bool is_positioned_behind = (behind_proj >= 45.0 && lateral_err <= 90.0);
    if (is_positioned_behind && !plan_locked_) {
        plan_locked_ = true;
        locked_plan_ = plan;
    }

    double target_x = aim_ball_x;
    double target_y = aim_ball_y;
    double target_w = clear_angle;
    uint8_t planner_type = oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE;

    if (direct_intercept) {
        // Interceptação direta da trajetória: entra na frente da bola em linha reta, sem dar volta
        target_x = aim_ball_x - clear_dir_x * 40.0;
        target_y = aim_ball_y - clear_dir_y * 40.0;
        target_w = angle_to_ball;
        planner_type = oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE;
    } else if (behind_proj >= 20.0) {
        // Já está do lado do nosso gol em relação à bola: avança direto em linha reta até encostar
        // Rampa suave de 130 mm -> 45 mm conforme reduz erro lateral (nunca trava a 180 mm)
        const double alpha = std::clamp((lateral_err - 45.0) / 110.0, 0.0, 1.0);
        const double offset_mm = 45.0 + alpha * (130.0 - 45.0);
        target_x = aim_ball_x - clear_dir_x * offset_mm;
        target_y = aim_ball_y - clear_dir_y * offset_mm;
        target_w = is_positioned_behind ? clear_angle : angle_to_ball;
        planner_type = oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE;
    } else {
        // Robô está à frente da bola (lado do adversário): contorna pelo lado mais curto no eixo
        // perpendicular a clear_dir, usando linha reta (que já faz via-point local) quando perto
        // para evitar travamentos do D* perto da bola/área.
        const double perp_x = -clear_dir_y;
        const double perp_y = clear_dir_x;
        const double side_proj = (rx - aim_ball_x) * perp_x + (ry - aim_ball_y) * perp_y;
        const double detour_side = (side_proj >= 0.0) ? 1.0 : -1.0;
        const double side_offset = (lateral_err < 140.0) ? 155.0 : 0.0;
        target_x = aim_ball_x - clear_dir_x * 130.0 + perp_x * detour_side * side_offset;
        target_y = aim_ball_y - clear_dir_y * 130.0 + perp_y * detour_side * side_offset;
        target_w = angle_to_ball;
        planner_type = (dist_to_ball < 700.0)
            ? oxebots_interfaces::msg::RobotGoal::PLANNER_STRAIGHT_LINE
            : oxebots_interfaces::msg::RobotGoal::PLANNER_DSTAR;
    }

    // Clamp obrigatório nos limites do campo e FORA da nossa pequena área (R-22 / SC-17)
    target_x = std::clamp(target_x, -max_fx, max_fx);
    target_y = std::clamp(target_y, -max_fy, max_fy);
    clampOutsideOwnArea(target_x, target_y, my_goal_x, penalty_depth, penalty_width);

    // Debounce + heartbeat de 200 ms (evita replanejamento a 60 Hz e renova o watchdog de 500 ms)
    const double now_s = ctx.node ? ctx.node->now().seconds() : 0.0;
    const bool target_changed =
        (std::abs(target_x - last_target_x_) > 5.0) ||
        (std::abs(target_y - last_target_y_) > 5.0) ||
        (std::abs(normalizeAngleRad(target_w - last_target_w_)) > 0.03) ||
        (planner_type != last_planner_type_);
    const bool heartbeat_due = (now_s - last_goal_publish_s_) >= 0.2;

    if (target_changed || heartbeat_due) {
        sendToController(Pose2D{target_x, target_y, target_w}, planner_type, true);
        last_target_x_ = target_x;
        last_target_y_ = target_y;
        last_target_w_ = target_w;
        last_planner_type_ = planner_type;
        last_goal_publish_s_ = now_s;
    }

    // Critério de liberação do chute (alinhado com a distância física ~140-150 mm e cone do kicker)
    const double kicker_ball_err = std::abs(normalizeAngleRad(angle_to_ball - ryaw));
    const double kick_dir_err = std::abs(normalizeAngleRad(clear_angle - ryaw));

    constexpr double kKickDistMm = 150.0;
    constexpr double kMaxKickerBallErrRad = 0.35;  // ~20 graus
    constexpr double kMaxPassDirErrRad = 0.24;     // ~13.7 graus
    constexpr double kMaxClearDirErrRad = 0.52;    // ~30 graus (bote rápido ao afastar)
    constexpr double kAlignTimeoutS = 0.25;

    const bool contact_ready =
        (real_behind_proj > 15.0) &&
        (dist_to_ball <= kKickDistMm) &&
        (kicker_ball_err < kMaxKickerBallErrRad);

    if (!contact_ready) {
        contact_since_valid_ = false;
    } else if (!contact_since_valid_) {
        contact_since_valid_ = true;
        contact_since_s_ = now_s;
    }

    const double contact_time = contact_since_valid_ ? std::max(0.0, now_s - contact_since_s_) : 0.0;
    double max_dir_err = (plan.pass_id >= 0) ? kMaxPassDirErrRad : kMaxClearDirErrRad;
    if (contact_time > kAlignTimeoutS) {
        max_dir_err *= 2.0;
    }

    if (contact_ready && kick_dir_err < max_dir_err) {
        const double final_speed = std::clamp(plan.speed, 0.5, 3.0);
        setOutput("kick_speed", final_speed);
        if (ctx.node) {
            if (plan.pass_id >= 0) {
                RCLCPP_INFO(ctx.node->get_logger(),
                    "DefenderClear (robô %u): PASSANDO para aliado %d (dist=%.0fmm, vel=%.2fm/s)",
                    self_id, plan.pass_id, dist_to_ball, final_speed);
            } else {
                RCLCPP_INFO(ctx.node->get_logger(),
                    "DefenderClear (robô %u): AFASTANDO bola (dist=%.0fmm, vel=%.2fm/s)",
                    self_id, dist_to_ball, final_speed);
            }
        }
        plan_locked_ = false;
        contact_since_valid_ = false;
        return BT::NodeStatus::SUCCESS;
    }

    if (ctx.node) {
        RCLCPP_INFO_THROTTLE(ctx.node->get_logger(), *ctx.node->get_clock(), 1000,
            "DefenderClear (robô %u): %s dist=%.0fmm atras=%.0fmm lateral=%.0fmm plano=%s",
            self_id, direct_intercept ? "INTERCEPTANDO" : "atacando bola",
            dist_to_ball, real_behind_proj, lateral_err,
            plan.pass_id >= 0 ? "passe" : "afastar");
    }

    return BT::NodeStatus::RUNNING;
}

// ---------------------------------------------------------------------------
// Ação: GoToClamped
// Estágio 2.2 — variante de GoToPoint com clamp de pequena área.
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
