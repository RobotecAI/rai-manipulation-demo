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

#include <memory>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "robotic_manipulation/arm_controller.h"
#include "robotic_manipulation/rai_manipulation_interface_node.h"

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto armController = std::make_shared<ArmController>();
  armController->Initialize();

  double constexpr EndEffectorBaseX = 0.3;
  double constexpr EndEffectorBaseY = 0.0;
  double constexpr EndEffectorBaseZ = 0.35;

  armController->MoveThroughWaypoints(
    {armController->CalculatePose(EndEffectorBaseX, EndEffectorBaseY, EndEffectorBaseZ)});

  std::vector<double> startingPose;
  startingPose = armController->CaptureJointValues();

  armController->SetJointValues(startingPose);
  armController->Close();

  {
    RaiManipulationInterfaceNode raiInterface;
    raiInterface.Initialize(*armController);
    raiInterface.Spin();
  }

  armController->SetJointValues(startingPose);

  // Shutdown ROS
  rclcpp::shutdown();

  return 0;
}