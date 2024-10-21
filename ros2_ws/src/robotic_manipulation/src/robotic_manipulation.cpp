#include <memory>

#include "robotic_manipulation/arm_controller.h"
#include "robotic_manipulation/state_controller.h"

#include <std_msgs/msg/float32_multi_array.hpp>

int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto armController = std::make_shared<ArmController>();

  armController->MoveThroughWaypoints({armController->CalculatePose(0.3, 0.0, 0.35)});

  std::vector<double> startingPose;
  startingPose = armController->CaptureJointValues();

  armController->SetJointValues(startingPose);

  StateController state;
  state.Begin(*armController);

  armController->SetJointValues(startingPose);

  // Shutdown ROS
  rclcpp::shutdown();

  return 0;
}