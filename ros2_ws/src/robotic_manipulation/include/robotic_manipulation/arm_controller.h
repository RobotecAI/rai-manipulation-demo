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

#include <moveit/move_group_interface/move_group_interface.h>

class ArmController
{
public:
  ArmController() = default;
  ~ArmController();

  void Initialize();

  geometry_msgs::msg::Pose CalculatePose(double x, double y, double z, double r = 0.0);

  bool MoveThroughWaypoints(std::vector<geometry_msgs::msg::Pose> const & waypoints);

  void Open();
  void Close();

  std::vector<double> GetEffectorPose();
  bool GetGripper();

  std::vector<double> CaptureJointValues();
  bool SetJointValues(std::vector<double> const & jointValues);

  void SetReferenceFrame(std::string const & frame);

private:
  // The joint values for the gripper to be open and closed.
  static double constexpr OpenGripperJointValue = 0.038;
  static double constexpr ClosedGripperJointValue = 0.002;

  // The base orientation of the end effector resulting in the gripper pointing
  // straight down.
  static double constexpr EndEffectorBaseRoll = 0.0;
  static double constexpr EndEffectorBasePitch = std::numbers::pi;
  static double constexpr EndEffectorBaseYaw = 45.0 * std::numbers::pi / 180.0;

  void WaitForClockMessage();

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_pandaArm;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_hand;

  std::atomic_bool gripper = false;

  rclcpp::Node::SharedPtr m_node;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;
};
