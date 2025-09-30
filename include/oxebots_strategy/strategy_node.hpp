#pragma once
#include <rclcpp/rclcpp.hpp>

namespace oxebots_strategy {

class StrategyNode : public rclcpp::Node {
public:
  explicit StrategyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
  rclcpp::TimerBase::SharedPtr timer_;
  void tick();
};

} // namespace oxebots_strategy
