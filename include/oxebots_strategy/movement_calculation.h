#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/robot_motion_override.hpp"
#include "oxebots_interfaces/msg/robot_motion_status.hpp"
#include "nav_msgs/msg/path.hpp"

#include <optional>
#include <vector>
#include <mutex>
#include <cmath>
#include <chrono>

namespace movement {
    struct Coordinate {
        float x = 0.0f;
        float y = 0.0f;
        float orientation = 0.0f;
    };

    inline double calculateDistance(const Coordinate& a, const Coordinate& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }
}

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode();

private:
    void game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
    void goal_callback(const oxebots_interfaces::msg::RobotGoal::SharedPtr msg);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void override_callback(const oxebots_interfaces::msg::RobotMotionOverride::SharedPtr msg);
    void calculate_and_move();

    // Único dono de /robot_commands para este robô (ver comentário grande em calculate_and_move()
    // sobre por que AlignToBall/KickBall/Halt pararam de publicar lá diretamente). Estes três
    // métodos tratam os modos que chegam via /robot_motion_override; a lógica de seguir caminho
    // (D*/linha reta + apontar para o gol) permanece inalterada, no corpo de calculate_and_move(),
    // e roda sempre que não há override ativo (modo NONE, incluindo por expiração de TTL).
    void runAlign(const movement::Coordinate& current_pos, uint8_t mode);
    void runKick();
    void publishHalt();
    void publishStatus(uint8_t active_mode, bool aligned);

    rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<oxebots_interfaces::msg::RobotMotionStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotMotionOverride>::SharedPtr override_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex data_mutex_;
    std::optional<movement::Coordinate> target_goal_;
    nav_msgs::msg::Path::SharedPtr last_path_;
    std::optional<double> target_w_;

    oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
    int robot_id_ = 0;
    bool game_data_received_ = false;

    bool is_yellow_team_;

    // --- Estado do canal de override (Align/Kick/Halt) ---
    // Se nenhum override for recebido por kOverrideTtl, o modo efetivo cai para NONE sozinho:
    // cobre o caso de um leaf (ex: Halt) que simplesmente para de ser ticado pelo BT sem nenhum
    // callback de "estou saindo" para avisar o controlador.
    static constexpr std::chrono::milliseconds kOverrideTtl{200};
    oxebots_interfaces::msg::RobotMotionOverride::SharedPtr last_override_msg_;
    std::chrono::steady_clock::time_point last_override_time_;

    // Estado da máquina de chute, portado de KickBallNode: dispara por kKickFireWindow, depois
    // aplica kKickCooldown antes de aceitar outro pedido. Persiste entre ativações (não é
    // resetado quando o override volta a NONE), já que o cooldown só importa para o PRÓXIMO
    // pedido de chute, não para o modo atualmente ativo.
    enum class KickPhase { IDLE, FIRING };
    static constexpr std::chrono::milliseconds kKickFireWindow{200};
    static constexpr std::chrono::milliseconds kKickCooldown{600};
    KickPhase kick_phase_ = KickPhase::IDLE;
    std::chrono::steady_clock::time_point kick_start_time_;
    std::chrono::steady_clock::time_point kick_cooldown_until_;
    uint32_t kick_fire_count_ = 0;
};
