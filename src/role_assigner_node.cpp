#include <rclcpp/rclcpp.hpp>
#include <oxebots_interfaces/msg/game_data.hpp>
#include <oxebots_interfaces/msg/role_assignment.hpp>
#include <cmath>
#include <algorithm>

class RoleAssignerNode : public rclcpp::Node
{
public:
  RoleAssignerNode() : Node("role_assigner_node")
  {
    this->declare_parameter<int>("goalkeeper_id", 0);
    this->declare_parameter<double>("hysteresis_threshold", 300.0); // mm

    goalkeeper_id_ = this->get_parameter("goalkeeper_id").as_int();
    hysteresis_threshold_ = this->get_parameter("hysteresis_threshold").as_double();

    role_pub_ = this->create_publisher<oxebots_interfaces::msg::RoleAssignment>("/role_assignment", 10);
    game_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
      "game_data", 10, std::bind(&RoleAssignerNode::game_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Role Assigner iniciado. Goleiro ID: %d, Histerese: %.1fmm", 
                goalkeeper_id_, hysteresis_threshold_);
  }

private:
  void game_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
  {
    std::vector<uint32_t> eligible_ids;
    std::map<uint32_t, double> distances;

    for (const auto & robot : msg->robots.allies) {
      if (static_cast<int>(robot.id) == goalkeeper_id_) continue;

      double dx = robot.x - msg->ball.x;
      double dy = robot.y - msg->ball.y;
      distances[robot.id] = std::sqrt(dx*dx + dy*dy);
      eligible_ids.push_back(robot.id);
    }

    if (eligible_ids.empty()) return;

    uint32_t current_attacker = 0;
    uint32_t current_defender = 0;

    // Lógica de Atribuição com Histerese
    if (eligible_ids.size() == 1) {
      current_attacker = eligible_ids[0];
      current_defender = goalkeeper_id_; // Fallback
    } else {
      // Ordenar por distância
      std::sort(eligible_ids.begin(), eligible_ids.end(), [&](uint32_t a, uint32_t b) {
        return distances[a] < distances[b];
      });

      uint32_t best_candidate = eligible_ids[0];
      uint32_t second_best = eligible_ids[1];

      // Se já temos um atacante e ele ainda está "perto o suficiente" (histerese)
      if (last_attacker_id_ != 999 && distances.count(last_attacker_id_)) {
        double dist_to_last = distances[last_attacker_id_];
        double dist_to_best = distances[best_candidate];

        if (best_candidate != last_attacker_id_ && dist_to_best > dist_to_last - hysteresis_threshold_) {
          current_attacker = last_attacker_id_;
          current_defender = best_candidate;
        } else {
          current_attacker = best_candidate;
          current_defender = second_best;
        }
      } else {
        current_attacker = best_candidate;
        current_defender = second_best;
      }
    }

    last_attacker_id_ = current_attacker;

    oxebots_interfaces::msg::RoleAssignment out_msg;
    out_msg.attacker_id = current_attacker;
    out_msg.defender_id = current_defender;
    role_pub_->publish(out_msg);

    RCLCPP_DEBUG(this->get_logger(), "Roles: Atacante=%d, Defensor=%d", current_attacker, current_defender);
  }

  int goalkeeper_id_;
  double hysteresis_threshold_;
  uint32_t last_attacker_id_ = 999;
  rclcpp::Publisher<oxebots_interfaces::msg::RoleAssignment>::SharedPtr role_pub_;
  rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoleAssignerNode>());
  rclcpp::shutdown();
  return 0;
}
