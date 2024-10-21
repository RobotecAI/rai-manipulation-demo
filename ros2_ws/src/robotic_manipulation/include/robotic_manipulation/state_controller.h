#pragma once

#include "robotic_manipulation/arm_controller.h"

class StateController {
public:
  StateController();
  ~StateController();

  void Begin(ArmController &arm);

private:
  rclcpp::Node::SharedPtr m_node;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;
};