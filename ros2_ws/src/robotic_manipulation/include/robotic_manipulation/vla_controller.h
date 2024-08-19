#pragma once

#include "robotic_manipulation/arm_controller.h"

class VLAController {
public:
  VLAController();
  ~VLAController();

  void Begin(ArmController &arm);

private:
  rclcpp::Node::SharedPtr m_node;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;
};