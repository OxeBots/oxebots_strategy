#include "oxebots_strategy/strategy_node.hpp"
using namespace std::chrono_literals;

namespace oxebots_strategy {

StrategyNode::StrategyNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("strategy_node", options)
{
  declare_parameter<int>("robot_amount", 3);
  timer_ = create_wall_timer(500ms, std::bind(&StrategyNode::tick, this));
  RCLCPP_INFO(get_logger(), "StrategyNode started. robot_amount=%ld",
              get_parameter("robot_amount").as_int());
}

void StrategyNode::tick() {
  RCLCPP_DEBUG(get_logger(), "ticking strategy...");
}

} // namespace oxebots_strategy

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<oxebots_strategy::StrategyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
