#include "oxebots_strategy/update_ball_position_node.h"

namespace oxebots_strategy
{

UpdateBallPositionNode::UpdateBallPositionNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
{
  game_data_sub_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10, std::bind(&UpdateBallPositionNode::gameDataCallback, this, std::placeholders::_1));
  
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/attack_intent_marker", 10);

  RCLCPP_INFO(node_->get_logger(), "UpdateBallPositionNode configurado.");
}


BT::PortsList UpdateBallPositionNode::providedPorts()
{
  return { BT::OutputPort<double>("ball_x"), 
           BT::OutputPort<double>("ball_y"),
           BT::OutputPort<double>("pre_kick_x"),
           BT::OutputPort<double>("pre_kick_y"),
           BT::OutputPort<bool>("is_ready_to_kick"),
           BT::OutputPort<bool>("ball_in_enemy_half"),
           BT::InputPort<unsigned int>("robot_id"),
           BT::InputPort<double>("goal_x"),
           BT::InputPort<double>("goal_y") };
}

void UpdateBallPositionNode::gameDataCallback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_game_data_ = msg;
}

BT::NodeStatus UpdateBallPositionNode::onStart()
{
  return onRunning();
}

BT::NodeStatus UpdateBallPositionNode::onRunning()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!last_game_data_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Nenhum dado de jogo (last_game_data) recebido, aguardando...");
    return BT::NodeStatus::RUNNING;
  }
  
  double ball_x = last_game_data_->ball.x;
  double ball_y = last_game_data_->ball.y;

  setOutput("ball_x", ball_x);
  setOutput("ball_y", ball_y);
  setOutput("ball_in_enemy_half", (ball_x > 0.0));

  // --- Calcular Ponto Atrás da Bola (Pre-Kick) ---
  double goal_x, goal_y;
  unsigned int robot_id;
  if (getInput<double>("goal_x", goal_x) && getInput<double>("goal_y", goal_y)) {
    // Vetor do Gol para a Bola
    double dx = ball_x - goal_x;
    double dy = ball_y - goal_y;
    double dist_ball_goal = std::hypot(dx, dy);

    if (dist_ball_goal > 10.0) {
      // Ponto a 400mm da bola, na mesma linha do gol
      double pre_kick_dist = 400.0; 
      double pk_x = ball_x + (dx / dist_ball_goal) * pre_kick_dist;
      double pk_y = ball_y + (dy / dist_ball_goal) * pre_kick_dist;
      
      setOutput("pre_kick_x", pk_x);
      setOutput("pre_kick_y", pk_y);

      // --- Visualização RViz (Linha de Ataque) ---
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "map";
      marker.header.stamp = node_->now();
      marker.ns = "attack_intent";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.05; // Largura da linha
      marker.color.a = 0.5;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;

      geometry_msgs::msg::Point p_goal, p_ball, p_pk;
      p_goal.x = goal_x / 1000.0; p_goal.y = goal_y / 1000.0;
      p_ball.x = ball_x / 1000.0; p_ball.y = ball_y / 1000.0;
      p_pk.x = pk_x / 1000.0; p_pk.y = pk_y / 1000.0;

      marker.points.push_back(p_goal);
      marker.points.push_back(p_ball);
      marker.points.push_back(p_pk);
      marker_pub_->publish(marker);

      // --- Verificar se o robô já está "atrás da bola" ---
      if (getInput<unsigned int>("robot_id", robot_id)) {
        bool found = false;
        for (const auto& ally : last_game_data_->robots.allies) {
          if (ally.id == robot_id) {
            found = true;
            double rx = ally.x;
            double ry = ally.y;

            double d_pk_x = pk_x - rx;
            double d_pk_y = pk_y - ry;
            double dist_to_pk = std::hypot(d_pk_x, d_pk_y);

            // HISTERESE: 
            // Se não estava pronto, precisa chegar a 150mm.
            // Se já estava pronto, pode ficar até 600mm (permitindo o avanço do chute).
            bool current_ready = false;
            (void)config().blackboard->get("is_ready_to_kick", current_ready);

            bool is_ready;
            if (!current_ready) {
                is_ready = (dist_to_pk < 150.0);
            } else {
                is_ready = (dist_to_pk < 600.0);
            }

            setOutput("is_ready_to_kick", is_ready);
            // Log de depuração
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, 
                "DEBUG Robot %u: Dist to PK: %.1f mm | Ready: %s", 
                robot_id, dist_to_pk, is_ready ? "SIM" : "NAO");

            // --- Visualização RViz (Status Ready) ---
            auto ready_marker = visualization_msgs::msg::Marker();
            ready_marker.header.frame_id = "map";
            ready_marker.header.stamp = node_->now();
            ready_marker.ns = "attack_ready";
            ready_marker.id = (int)robot_id;
            ready_marker.type = visualization_msgs::msg::Marker::SPHERE;
            ready_marker.action = visualization_msgs::msg::Marker::ADD;
            ready_marker.pose.position.x = rx / 1000.0;
            ready_marker.pose.position.y = ry / 1000.0;
            ready_marker.pose.position.z = 0.3; // Flutua sobre o robô
            ready_marker.scale.x = 0.1;
            ready_marker.scale.y = 0.1;
            ready_marker.scale.z = 0.1;
            ready_marker.color.a = 0.8;
            if (is_ready) {
                ready_marker.color.r = 0.0; ready_marker.color.g = 1.0; ready_marker.color.b = 0.0; // Verde
            } else {
                ready_marker.color.r = 1.0; ready_marker.color.g = 1.0; ready_marker.color.b = 0.0; // Amarelo
            }
            marker_pub_->publish(ready_marker);
            break;
          }
        }
        if (!found) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                "Robo %u nao encontrado na lista de aliados!", robot_id);
        }
      }
    }
  }
  
  return BT::NodeStatus::SUCCESS;
}

void UpdateBallPositionNode::onHalted()
{
}

} // namespace oxebots_strategy
