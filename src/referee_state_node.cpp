// Copyright 2024 Oxebots
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/game_state.hpp"
#include "ssl_league_msgs/msg/referee.hpp"

class RefereeStateNode : public rclcpp::Node
{
public:
    using GameState = oxebots_interfaces::msg::GameState;
    using Referee = ssl_league_msgs::msg::Referee;
    using BallPosition = oxebots_interfaces::msg::BallPosition;

    RefereeStateNode()
    : Node("referee_state_node")
    {
        this->declare_parameter<bool>("is_yellow_team", false);
        this->declare_parameter<std::string>("gc_topic", "/gc_multicast_bridge/referee_messages");
        this->declare_parameter<std::string>("ball_topic", "ball_data");
        this->declare_parameter<double>("ball_moved_mm", 50.0);
        this->declare_parameter<double>("kick_timeout_s", 10.0);
        this->declare_parameter<double>("stop_speed_mps", 0.75);
        this->declare_parameter<double>("play_speed_mps", 1.5);

        is_yellow_team_ = this->get_parameter("is_yellow_team").as_bool();
        ball_moved_mm_ = this->get_parameter("ball_moved_mm").as_double();
        kick_timeout_s_ = this->get_parameter("kick_timeout_s").as_double();
        stop_speed_mps_ = this->get_parameter("stop_speed_mps").as_double();
        play_speed_mps_ = this->get_parameter("play_speed_mps").as_double();

        const auto gc_topic = this->get_parameter("gc_topic").as_string();
        const auto ball_topic = this->get_parameter("ball_topic").as_string();

        // QoS reliable + transient_local para que late-joiners recebam o último GameState
        auto state_qos = rclcpp::QoS(1).transient_local().reliable();
        state_pub_ = this->create_publisher<GameState>("/game_state", state_qos);

        gc_sub_ = this->create_subscription<Referee>(
            gc_topic, 10, std::bind(&RefereeStateNode::refereeCallback, this, std::placeholders::_1));

        ball_sub_ = this->create_subscription<BallPosition>(
            ball_topic, 10, std::bind(&RefereeStateNode::ballCallback, this, std::placeholders::_1));

        initDefaultState();
        publishState();

        RCLCPP_INFO(
            this->get_logger(),
            "referee_state_node iniciado (is_yellow_team=%s, gc_topic=%s).",
            is_yellow_team_ ? "true" : "false", gc_topic.c_str());
    }

private:
    void initDefaultState()
    {
        state_.command_counter = 0;
        state_.command = Referee::COMMAND_HALT;
        state_.phase = GameState::PHASE_HALT;
        state_.restart_ctx = GameState::RESTART_NONE;
        state_.we_favored = false;
        state_.stage = 0;
        state_.our_goal_positive_x = is_yellow_team_;
        state_.has_designated = false;
        state_.designated_position.x = 0.0f;
        state_.designated_position.y = 0.0f;
        state_.designated_position.z = 0.0f;
        state_.speed_limit_mps = 0.0f;
        state_.ball_min_distance_mm = 0.0f;
        state_.can_place_ball = false;
        state_.max_allowed_bots = 3;
        state_.our_goalkeeper_id = 0;
        state_.stale = true;
    }

    void refereeCallback(const Referee::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        // 1. Descartar mensagens fora de ordem / antigas (SC-21)
        if (has_received_gc_ && msg->command_counter < last_command_counter_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Mensagem do GC ignorada (contador antigo: %u < %u).",
                msg->command_counter, last_command_counter_);
            return;
        }

        const bool is_new_command = (!has_received_gc_ || msg->command_counter > last_command_counter_);
        has_received_gc_ = true;
        last_command_counter_ = msg->command_counter;

        state_.header.stamp = this->now();
        state_.command_counter = msg->command_counter;
        state_.command = msg->command;
        state_.stage = msg->stage;
        state_.stale = false;

        // 5. Resolver o lado do campo a partir de blue_team_on_positive_half e is_yellow_team
        if (!msg->blue_team_on_positive_half.empty())
        {
            const bool blue_positive = msg->blue_team_on_positive_half[0];
            state_.our_goal_positive_x = is_yellow_team_ ? !blue_positive : blue_positive;
        }
        else
        {
            state_.our_goal_positive_x = is_yellow_team_;
        }

        // Posição designada (ssl_ros_bridge envia em metros; GameState armazena em mm)
        if (!msg->designated_position.empty())
        {
            state_.has_designated = true;
            state_.designated_position.x = msg->designated_position[0].x * 1000.0f;
            state_.designated_position.y = msg->designated_position[0].y * 1000.0f;
            state_.designated_position.z = 0.0f;
        }
        else
        {
            state_.has_designated = false;
            state_.designated_position.x = 0.0f;
            state_.designated_position.y = 0.0f;
            state_.designated_position.z = 0.0f;
        }

        // Extrair informações do nosso TeamInfo (can_place_ball, max_allowed_bots, our_goalkeeper_id)
        updateTeamInfo(*msg);

        // 2 & 3. Mapear comando -> fase e atualizar contexto de reinício (apenas quando command_counter avança)
        if (is_new_command)
        {
            applyCommandTransition(msg->command);
        }
        else
        {
            // Mesmo command_counter: verifica se o prazo de 10 s de KICKOFF/FREE_KICK venceu
            checkRestartTimeout();
        }

        // 4. Derivar limites regulamentares (speed_limit_mps e ball_min_distance_mm)
        updateDerivedLimits();

        if (is_new_command)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "GC cmd=%u (counter=%u) -> phase=%u, restart_ctx=%u, we_favored=%s, "
                "speed_limit=%.2f m/s, ball_min_dist=%.0f mm, can_place=%s, our_goal_pos_x=%s",
                state_.command, state_.command_counter, state_.phase, state_.restart_ctx,
                state_.we_favored ? "sim" : "nao",
                state_.speed_limit_mps, state_.ball_min_distance_mm,
                state_.can_place_ball ? "sim" : "nao",
                state_.our_goal_positive_x ? "sim" : "nao");
        }

        publishState();
    }

    void updateTeamInfo(const Referee & msg)
    {
        const auto & our_team = is_yellow_team_ ? msg.yellow : msg.blue;
        state_.our_goalkeeper_id = our_team.goalkeeper;
        state_.can_place_ball = !our_team.can_place_ball.empty() ? our_team.can_place_ball[0] : false;
        state_.max_allowed_bots = !our_team.max_allowed_bots.empty()
            ? static_cast<uint8_t>(std::min<uint32_t>(our_team.max_allowed_bots[0], 255U))
            : 3U;
    }

    void updateDerivedLimits()
    {
        constexpr float kStopBallMinDistanceMm = 500.0f;

        switch (state_.phase)
        {
            case GameState::PHASE_HALT:
            case GameState::PHASE_TIMEOUT:
                state_.speed_limit_mps = 0.0f;
                state_.ball_min_distance_mm = 0.0f;
                break;

            case GameState::PHASE_STOP:
            case GameState::PHASE_PREPARE_KICKOFF:
            case GameState::PHASE_PREPARE_PENALTY:
                state_.speed_limit_mps = static_cast<float>(stop_speed_mps_);
                state_.ball_min_distance_mm = kStopBallMinDistanceMm;
                break;

            case GameState::PHASE_BALL_PLACEMENT:
                state_.speed_limit_mps = static_cast<float>(stop_speed_mps_);
                state_.ball_min_distance_mm =
                    (state_.we_favored && state_.can_place_ball) ? 0.0f : kStopBallMinDistanceMm;
                break;

            case GameState::PHASE_KICKOFF:
            case GameState::PHASE_FREE_KICK:
                state_.speed_limit_mps = static_cast<float>(play_speed_mps_);
                state_.ball_min_distance_mm = state_.we_favored ? 0.0f : kStopBallMinDistanceMm;
                break;

            case GameState::PHASE_PENALTY:
            case GameState::PHASE_RUN:
                state_.speed_limit_mps = static_cast<float>(play_speed_mps_);
                state_.ball_min_distance_mm = 0.0f;
                break;

            default:
                state_.speed_limit_mps = 0.0f;
                state_.ball_min_distance_mm = 0.0f;
                break;
        }
    }

    void applyCommandTransition(uint8_t cmd)
    {
        const uint8_t prev_ctx = state_.restart_ctx;
        const bool prev_favored = state_.we_favored;

        switch (cmd)
        {
            case Referee::COMMAND_HALT:  // 0
                state_.phase = GameState::PHASE_HALT;
                state_.restart_ctx = prev_ctx;
                state_.we_favored = prev_favored;
                clearRestartTracking();
                break;

            case Referee::COMMAND_STOP:  // 1
                state_.phase = GameState::PHASE_STOP;
                state_.restart_ctx = prev_ctx;
                state_.we_favored = prev_favored;
                clearRestartTracking();
                break;

            case Referee::COMMAND_NORMAL_START:  // 2
                if (prev_ctx == GameState::RESTART_KICKOFF)
                {
                    state_.phase = GameState::PHASE_KICKOFF;
                    state_.restart_ctx = GameState::RESTART_KICKOFF;
                    state_.we_favored = prev_favored;
                    startRestartTracking();
                }
                else if (prev_ctx == GameState::RESTART_PENALTY)
                {
                    state_.phase = GameState::PHASE_PENALTY;
                    state_.restart_ctx = GameState::RESTART_PENALTY;
                    state_.we_favored = prev_favored;
                    clearRestartTracking();
                }
                else
                {
                    state_.phase = GameState::PHASE_RUN;
                    state_.restart_ctx = GameState::RESTART_NONE;
                    state_.we_favored = false;
                    clearRestartTracking();
                }
                break;

            case Referee::COMMAND_FORCE_START:  // 3
                state_.phase = GameState::PHASE_RUN;
                state_.restart_ctx = GameState::RESTART_FORCE;
                state_.we_favored = false;
                clearRestartTracking();
                break;

            case Referee::COMMAND_PREPARE_KICKOFF_YELLOW:  // 4
                state_.phase = GameState::PHASE_PREPARE_KICKOFF;
                state_.restart_ctx = GameState::RESTART_KICKOFF;
                state_.we_favored = is_yellow_team_;
                clearRestartTracking();
                break;

            case Referee::COMMAND_PREPARE_KICKOFF_BLUE:  // 5
                state_.phase = GameState::PHASE_PREPARE_KICKOFF;
                state_.restart_ctx = GameState::RESTART_KICKOFF;
                state_.we_favored = !is_yellow_team_;
                clearRestartTracking();
                break;

            case Referee::COMMAND_PREPARE_PENALTY_YELLOW:  // 6
                state_.phase = GameState::PHASE_PREPARE_PENALTY;
                state_.restart_ctx = GameState::RESTART_PENALTY;
                state_.we_favored = is_yellow_team_;
                clearRestartTracking();
                break;

            case Referee::COMMAND_PREPARE_PENALTY_BLUE:  // 7
                state_.phase = GameState::PHASE_PREPARE_PENALTY;
                state_.restart_ctx = GameState::RESTART_PENALTY;
                state_.we_favored = !is_yellow_team_;
                clearRestartTracking();
                break;

            case Referee::COMMAND_DIRECT_FREE_YELLOW:    // 8
            case Referee::COMMAND_INDIRECT_FREE_YELLOW:  // 10 (legado -> FREE_KICK)
                state_.phase = GameState::PHASE_FREE_KICK;
                state_.restart_ctx = GameState::RESTART_FREE_KICK;
                state_.we_favored = is_yellow_team_;
                startRestartTracking();
                break;

            case Referee::COMMAND_DIRECT_FREE_BLUE:    // 9
            case Referee::COMMAND_INDIRECT_FREE_BLUE:  // 11 (legado -> FREE_KICK)
                state_.phase = GameState::PHASE_FREE_KICK;
                state_.restart_ctx = GameState::RESTART_FREE_KICK;
                state_.we_favored = !is_yellow_team_;
                startRestartTracking();
                break;

            case Referee::COMMAND_TIMEOUT_YELLOW:  // 12
                state_.phase = GameState::PHASE_TIMEOUT;
                state_.restart_ctx = prev_ctx;
                state_.we_favored = is_yellow_team_;
                clearRestartTracking();
                break;

            case Referee::COMMAND_TIMEOUT_BLUE:  // 13
                state_.phase = GameState::PHASE_TIMEOUT;
                state_.restart_ctx = prev_ctx;
                state_.we_favored = !is_yellow_team_;
                clearRestartTracking();
                break;

            case Referee::COMMAND_GOAL_YELLOW:  // 14 (legado -> STOP)
                state_.phase = GameState::PHASE_STOP;
                state_.restart_ctx = GameState::RESTART_NONE;
                state_.we_favored = is_yellow_team_;
                clearRestartTracking();
                break;

            case Referee::COMMAND_GOAL_BLUE:  // 15 (legado -> STOP)
                state_.phase = GameState::PHASE_STOP;
                state_.restart_ctx = GameState::RESTART_NONE;
                state_.we_favored = !is_yellow_team_;
                clearRestartTracking();
                break;

            case Referee::COMMAND_BALL_PLACEMENT_YELLOW:  // 16
                state_.phase = GameState::PHASE_BALL_PLACEMENT;
                state_.restart_ctx = prev_ctx;
                state_.we_favored = is_yellow_team_;
                clearRestartTracking();
                break;

            case Referee::COMMAND_BALL_PLACEMENT_BLUE:  // 17
                state_.phase = GameState::PHASE_BALL_PLACEMENT;
                state_.restart_ctx = prev_ctx;
                state_.we_favored = !is_yellow_team_;
                clearRestartTracking();
                break;

            default:
                RCLCPP_WARN(
                    this->get_logger(),
                    "Comando desconhecido do GC (%u); mantendo HALT por seguranca.", cmd);
                state_.phase = GameState::PHASE_HALT;
                state_.we_favored = false;
                clearRestartTracking();
                break;
        }
    }

    void startRestartTracking()
    {
        restart_start_time_ = this->now();
        if (last_ball_pos_.has_value())
        {
            restart_ball_origin_ = last_ball_pos_;
        }
        else
        {
            restart_ball_origin_.reset();
        }
    }

    void clearRestartTracking()
    {
        restart_start_time_.reset();
        restart_ball_origin_.reset();
    }

    void checkRestartTimeout()
    {
        if (state_.phase != GameState::PHASE_KICKOFF && state_.phase != GameState::PHASE_FREE_KICK)
        {
            return;
        }
        if (!restart_start_time_.has_value())
        {
            return;
        }
        const double elapsed_s = (this->now() - *restart_start_time_).seconds();
        if (elapsed_s >= kick_timeout_s_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Reinicio atingiu timeout (%.1f s) -> transicionando para PHASE_RUN.", elapsed_s);
            state_.phase = GameState::PHASE_RUN;
            state_.we_favored = false;
            clearRestartTracking();
        }
    }

    void ballCallback(const BallPosition::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        const std::pair<double, double> current_ball{msg->x, msg->y};
        last_ball_pos_ = current_ball;

        if (state_.phase != GameState::PHASE_KICKOFF && state_.phase != GameState::PHASE_FREE_KICK)
        {
            return;
        }

        if (!restart_ball_origin_.has_value())
        {
            restart_ball_origin_ = current_ball;
        }

        const double moved_mm = std::hypot(
            current_ball.first - restart_ball_origin_->first,
            current_ball.second - restart_ball_origin_->second);

        const bool timed_out = restart_start_time_.has_value() &&
            ((this->now() - *restart_start_time_).seconds() >= kick_timeout_s_);

        if (moved_mm >= ball_moved_mm_ || timed_out)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Bola em jogo (moved=%.1f mm, timeout=%s) -> transicionando para PHASE_RUN.",
                moved_mm, timed_out ? "sim" : "nao");
            state_.phase = GameState::PHASE_RUN;
            state_.we_favored = false;
            state_.header.stamp = this->now();
            clearRestartTracking();
            updateDerivedLimits();
            publishState();
        }
    }

    void publishState()
    {
        state_pub_->publish(state_);
    }

    bool is_yellow_team_{false};
    double ball_moved_mm_{50.0};
    double kick_timeout_s_{10.0};
    double stop_speed_mps_{0.75};
    double play_speed_mps_{1.5};

    bool has_received_gc_{false};
    uint32_t last_command_counter_{0};

    std::optional<std::pair<double, double>> last_ball_pos_;
    std::optional<std::pair<double, double>> restart_ball_origin_;
    std::optional<rclcpp::Time> restart_start_time_;

    GameState state_;

    rclcpp::Publisher<GameState>::SharedPtr state_pub_;
    rclcpp::Subscription<Referee>::SharedPtr gc_sub_;
    rclcpp::Subscription<BallPosition>::SharedPtr ball_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RefereeStateNode>());
    rclcpp::shutdown();
    return 0;
}
