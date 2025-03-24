/* Copyright (C) 2025 Robotec.AI
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "robotic_manipulation/arm_controller.h"

#include "rai_interfaces/srv/manipulator_move_to.hpp"
#include <std_srvs/srv/trigger.hpp>

class RaiManipulationInterfaceNode {
public:
  RaiManipulationInterfaceNode();

  void Initialize(ArmController &arm);
  void Spin();

private:
  rclcpp::Node::SharedPtr m_node;
  rclcpp::Service<rai_interfaces::srv::ManipulatorMoveTo>::SharedPtr
      m_moveToService;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_resetService;

  rclcpp::executors::SingleThreadedExecutor m_executor;

  std::vector<double> m_startingPose;
};