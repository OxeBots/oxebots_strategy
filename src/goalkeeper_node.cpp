#include "oxebots_strategy/goalkeeper_node.h"
#include <cmath>
#include <algorithm> 

namespace {
    // Constantes de controle (semelhantes ao GoToPoint)
    const double K_P = 2.0; 
    const double K_W = 3.0; 
    const double LOOKAHEAD_DISTANCE = 0.2;
    const double ROBOT_RADIUS = 0.09;
    const double STOP_THRESHOLD = 0.05; // 5cm de tolerância para o goleiro

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
} // namespace

namespace oxebots_strategy
{

GoalkeeperNode::GoalkeeperNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node_ptr)
  : BT::StatefulActionNode(name, config), node_(node_ptr), robot_data_updated_(false)
{
  if (!getInput<int>("robot_id", robot_id_)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing required input [robot_id] for GoalkeeperNode");
      throw BT::RuntimeError("Missing required input [robot_id]");
  }

  auto blackboard = config.blackboard;
  if (!blackboard->get("my_goal_x", my_goal_x_)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing 'my_goal_x' from blackboard for GoalkeeperNode");
      throw BT::RuntimeError("Missing 'my_goal_x' from blackboard");
  }
  if (!blackboard->get("is_yellow", is_yellow_team_)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing 'is_yellow' from blackboard for GoalkeeperNode");
      throw BT::RuntimeError("Missing 'is_yellow' from blackboard");
  }

  cmd_pub_ = node_->create_publisher<oxebots_interfaces::msg::RobotCmd>("/robot_commands", 10);
  
  game_data_subscription_ = node_->create_subscription<oxebots_interfaces::msg::GameData>(
    "/game_data", 10, std::bind(&GoalkeeperNode::game_data_callback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "GoalkeeperNode (RRT*) para o robô %d configurado. Meu gol em X: %.1f", robot_id_, my_goal_x_);
}

BT::PortsList GoalkeeperNode::providedPorts()
{
  return { BT::InputPort<int>("robot_id") };
}

void GoalkeeperNode::game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
  last_game_data_ = msg;
  robot_data_updated_ = true;
}

BT::NodeStatus GoalkeeperNode::onStart()
{
  if (robot_id_ != 0) {
    RCLCPP_ERROR(node_->get_logger(), "GoalkeeperNode deve ser atribuído apenas ao robô 0. ID atual: %d", robot_id_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoalkeeperNode::onRunning()
{
  if (!robot_data_updated_ || !last_game_data_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "GoalkeeperNode: Aguardando dados do jogo...");
    return BT::NodeStatus::RUNNING;
  }
  
  // Encontrar dados do robô atual
  std::optional<oxebots_interfaces::msg::RobotGameData> robot_opt;
  for(const auto& ally : last_game_data_->robots.allies){
      if(ally.id == (unsigned int)robot_id_){
          robot_opt = ally;
          break;
      }
  }

  if(!robot_opt){
      RCLCPP_WARN(node_->get_logger(), "Aguardando dados do robô %d...", robot_id_);
      return BT::NodeStatus::RUNNING;
  }

  auto robot_data = *robot_opt;
  Point start_pos = {robot_data.x / 1000.0, robot_data.y / 1000.0};
  double robot_w = robot_data.orientation;

  // --- Lógica do Goleiro ---
  double ball_y = last_game_data_->ball.y / 1000.0; // Convertendo para metros
  target_pos_.x = my_goal_x_ / 1000.0;
  target_pos_.y = std::clamp(ball_y, -0.4, 0.4); // Limita em metros
  target_w_ = normalizeAngle(std::atan2(0.0 - target_pos_.y, 0.0 - target_pos_.x));

  // --- Lógica do RRT* (adaptada do GoToPoint) ---
  double dist_to_target = std::hypot(start_pos.x - target_pos_.x, start_pos.y - target_pos_.y);
  if (dist_to_target < STOP_THRESHOLD) {
      // Perto o suficiente, apenas ajusta a orientação
      auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
      oxebots_interfaces::msg::RobotCmdData robot_cmd;
      robot_cmd.id = robot_id_;
      robot_cmd.x_velocity = 0.0f;
      robot_cmd.y_velocity = 0.0f;
      robot_cmd.angular_velocity = K_W * normalizeAngle(target_w_ - robot_w);
      cmd_msg->robots.push_back(robot_cmd);
      cmd_pub_->publish(std::move(cmd_msg));
      return BT::NodeStatus::RUNNING; // Goleiro nunca "termina"
  }

  std::vector<Obstacle> obstacles;
  for (const auto& ally : last_game_data_->robots.allies) {
    if (ally.id != (unsigned int)robot_id_) obstacles.push_back({{ally.x / 1000.0, ally.y / 1000.0}, ROBOT_RADIUS});
  }
  for (const auto& enemy : last_game_data_->robots.enemies) {
    obstacles.push_back({{enemy.x / 1000.0, enemy.y / 1000.0}, ROBOT_RADIUS});
  }

  RRTStarPlanner planner(start_pos, target_pos_, obstacles, 0.15, 0.3, 0.5, 1000); // Menos iterações para o goleiro
  current_path_ = planner.plan_path();

  auto cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
  oxebots_interfaces::msg::RobotCmdData robot_cmd;
  robot_cmd.id = robot_id_;

  if (current_path_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Goleiro (RRT*): Nenhum caminho encontrado. Parando robô %d.", robot_id_);
    robot_cmd.x_velocity = 0.0f;
    robot_cmd.y_velocity = 0.0f;
    robot_cmd.angular_velocity = 0.0f;
  } else {
    lookahead_point_ = find_lookahead_point(current_path_, start_pos, LOOKAHEAD_DISTANCE);
    
    double dx_global = lookahead_point_.x - start_pos.x;
    double dy_global = lookahead_point_.y - start_pos.y;
    
    robot_cmd.x_velocity = K_P * dx_global;
    robot_cmd.y_velocity = K_P * dy_global;
    robot_cmd.angular_velocity = K_W * normalizeAngle(target_w_ - robot_w);
  }
    
  cmd_msg->robots.push_back(robot_cmd);
  cmd_pub_->publish(std::move(cmd_msg));
  return BT::NodeStatus::RUNNING;
}

void GoalkeeperNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "GoalkeeperNode (RRT*) parado. Robô %d", robot_id_);
  auto stop_cmd_msg = std::make_unique<oxebots_interfaces::msg::RobotCmd>();
  oxebots_interfaces::msg::RobotCmdData robot_cmd;
  robot_cmd.id = robot_id_;
  robot_cmd.x_velocity = 0.0f;
  robot_cmd.y_velocity = 0.0f;
  robot_cmd.angular_velocity = 0.0f;
  cmd_pub_->publish(std::move(stop_cmd_msg));
}

Point GoalkeeperNode::find_lookahead_point(const std::vector<Point>& path, const Point& robot_pos, double lookahead_distance) {
    if (path.empty()) return robot_pos;

    Point lookahead_pt = path.back();
    if (path.size() > 1) {
        for (size_t i = path.size() - 1; i > 0; --i) {
            Point p1 = path[i-1];
            Point p2 = path[i];
            
            Point d = {p2.x - p1.x, p2.y - p1.y};
            Point f = {p1.x - robot_pos.x, p1.y - robot_pos.y};

            double a = d.x*d.x + d.y*d.y;
            double b = 2 * (f.x*d.x + f.y*d.y);
            double c = f.x*f.x + f.y*f.y - lookahead_distance*lookahead_distance;
            double discriminant = b*b - 4*a*c;

            if (discriminant >= 0) {
                discriminant = sqrt(discriminant);
                double t1 = (-b - discriminant) / (2*a);
                double t2 = (-b + discriminant) / (2*a);
                
                if (t1 >= 0 && t1 <= 1) return {p1.x + t1*d.x, p1.y + t1*d.y};
                if (t2 >= 0 && t2 <= 1) return {p1.x + t2*d.x, p1.y + t2*d.y};
            }
        }
    }
    return lookahead_pt;
}

}  // namespace oxebots_strategy
