#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "oxebots_interfaces/msg/robot_goal.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"

#include <optional>
#include <vector>
#include <mutex>
#include <cmath>

namespace movement {
    // Definição da estrutura Coordinate
    struct Coordinate {
        float x = 0.0f;
        float y = 0.0f;
        float orientation = 0.0f;
    };

    // Definição da classe PotentialField
    class PotentialField {
    public:
        // <<< MUDANÇA DRÁSTICA AQUI: Física totalmente refeita
        std::vector<double> calculate(
            const Coordinate& current_pos, 
            const Coordinate& target_pos, 
            const std::vector<Coordinate>& obstacles,
            double k_att,  // Ganho atrativo (magnitude constante)
            double k_rep,  // Ganho repulsivo
            double d0_rep  // Raio de influência repulsivo (ex: 500mm)
        ) {
            
            // --- 1. FORÇA ATRATIVA (Normalizada) ---
            // A força atrativa agora tem magnitude constante = k_att
            
            double vec_att_x = target_pos.x - current_pos.x;
            double vec_att_y = target_pos.y - current_pos.y;
            double dist_to_target = std::hypot(vec_att_x, vec_att_y);

            double attractive_force_x = 0.0;
            double attractive_force_y = 0.0;

            // Evita divisão por zero se já estivermos no alvo
            if (dist_to_target > 1.0) { 
                attractive_force_x = k_att * (vec_att_x / dist_to_target);
                attractive_force_y = k_att * (vec_att_y / dist_to_target);
            }

            // --- 2. FORÇA REPULSIVA (Nova Fórmula) ---
            // F_rep = k_rep * (d0 - d) / d
            // Esta fórmula é adimensional e cresce muito perto do obstáculo
            
            double repulsive_force_x = 0.0;
            double repulsive_force_y = 0.0;

            for (const auto& obs : obstacles) {
                double vec_obs_x = current_pos.x - obs.x;
                double vec_obs_y = current_pos.y - obs.y;
                double dist = std::hypot(vec_obs_x, vec_obs_y);

                // d0_rep é a distância de centro a centro
                if (dist < d0_rep && dist > 1.0) { 
                    
                    // Fórmula: k_rep * (1/d - 1/d0) * (1/d^2) é muito pequena
                    // Vamos usar uma fórmula mais simples e forte:
                    // magnitude = k_rep * (d0_rep - dist) / dist
                    // (Esta fórmula é adimensional e cresce muito rápido perto de dist=0)
                    
                    double force_mag = k_rep * (d0_rep - dist) / dist;

                    // Normaliza o vetor do obstáculo para o robô
                    double unit_vec_x = vec_obs_x / dist;
                    double unit_vec_y = vec_obs_y / dist;
                    
                    repulsive_force_x += force_mag * unit_vec_x;
                    repulsive_force_y += force_mag * unit_vec_y;
                }
            }

            // --- 3. FORÇA TOTAL ---
            double total_force_x = attractive_force_x + repulsive_force_x;
            double total_force_y = attractive_force_y + repulsive_force_y;
            double target_angle = std::atan2(total_force_y, total_force_x);

            // Retorna o VETOR de força total. O nó irá normalizar isso.
            return {total_force_x, total_force_y, target_angle};
        }
    };

    inline double calculateDistance(const Coordinate& a, const Coordinate& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }
} // namespace movement

class PotentialFieldNode : public rclcpp::Node {
public:
    PotentialFieldNode();

private:
    void game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
    
    void goal_callback(const oxebots_interfaces::msg::RobotGoal::SharedPtr msg);

    void calculate_and_move();

    rclcpp::Publisher<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
    
    rclcpp::Subscription<oxebots_interfaces::msg::RobotGoal>::SharedPtr goal_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex data_mutex_;
    std::optional<movement::Coordinate> target_pos_;
    
    std::optional<double> target_w_; 

    oxebots_interfaces::msg::GameData::SharedPtr last_game_data_;
    unsigned int robot_id_ = 0; 
    bool game_data_received_ = false;

    // Variáveis para a lógica de restrição de área
    bool is_yellow_team_;
    double my_goal_x_;
    double opponent_goal_x_;
};