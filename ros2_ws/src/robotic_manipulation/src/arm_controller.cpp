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

#include "robotic_manipulation/arm_controller.h"

#include <rosgraph_msgs/msg/clock.hpp>

#include <numbers>

void ArmController::Initialize() {
  m_node = rclcpp::Node::make_shared("arm_controller");
  m_node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  WaitForClockMessage();

  m_executor.add_node(m_node);
  m_spinner = std::thread([this]() { m_executor.spin(); });

  m_pandaArm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      m_node, "panda_arm");
  m_hand = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      m_node, "hand");

  m_pandaArm->setMaxVelocityScalingFactor(1.0);
  m_pandaArm->setMaxAccelerationScalingFactor(1.0);
}

ArmController::~ArmController() {
  m_executor.cancel();
  if (m_spinner.joinable()) {
    m_spinner.join();
  }
}

geometry_msgs::msg::Pose ArmController::CalculatePose(double x, double y,
                                                      double z, double r) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  auto quat = tf2::Quaternion();
  quat.setEuler(EndEffectorBaseRoll, EndEffectorBasePitch,
                EndEffectorBaseYaw + r);
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();

  return pose;
}

bool ArmController::MoveThroughWaypoints(
    std::vector<geometry_msgs::msg::Pose> const &waypoints) {
  auto logger = m_node->get_logger();

  int const NumTries = 10;
  for (int i = 0; i < NumTries; i++) {
    moveit_msgs::msg::RobotTrajectory trajectory;
    if (m_pandaArm->computeCartesianPath(waypoints, 0.01, 0.0, trajectory) ==
        -1) {
      RCLCPP_ERROR(logger,
                   "MoveThroughWaypoints: Failed to compute Cartesian path");
      continue;
    }

    if (m_pandaArm->execute(trajectory) ==
        moveit::core::MoveItErrorCode::SUCCESS) {
      return true;
    }
    RCLCPP_ERROR(logger, "MoveThroughWaypoints: Failed to execute "
                         "trajectory, trying again...");
  }

  RCLCPP_ERROR(
      logger,
      "MoveThroughWaypoints: Failed to execute trajectory after %d tries",
      NumTries);
  return false;
}

void ArmController::Open() {
  gripper.store(true);
  m_hand->setJointValueTarget("panda_finger_joint1", OpenGripperJointValue);
  while (m_hand->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(m_node->get_logger(), "Failed to open hand");
  }
}

void ArmController::Close() {
  gripper.store(false);
  m_hand->setJointValueTarget("panda_finger_joint1", ClosedGripperJointValue);
  while (m_hand->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(m_node->get_logger(), "Failed to close hand");
  }
}

std::vector<double> ArmController::GetEffectorPose() {
  auto pose = m_pandaArm->getCurrentPose().pose;
  auto rotation = tf2::Quaternion(pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3 m(rotation);
  m.getRPY(pose.orientation.x, pose.orientation.y, pose.orientation.z, 0);
  return {pose.position.x,
          pose.position.y,
          pose.position.z,
          pose.orientation.x,
          pose.orientation.y - EndEffectorBasePitch,
          pose.orientation.z - (tf2Radians(180.0) - EndEffectorBaseYaw)};
};

bool ArmController::GetGripper() { return gripper.load(); }

std::vector<double> ArmController::CaptureJointValues() {
  return m_pandaArm->getCurrentJointValues();
}

bool ArmController::SetJointValues(std::vector<double> const &jointValues) {
  m_pandaArm->setJointValueTarget(jointValues);
  if (m_pandaArm->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(m_node->get_logger(), "Failed to set joint values");
    return false;
  }
  return true;
}

void ArmController::SetReferenceFrame(std::string const &frame) {
  m_pandaArm->setPoseReferenceFrame(frame);
}

void ArmController::WaitForClockMessage() {
  bool clockReceived = false;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  auto subscription = m_node->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", qos,
      [&](rosgraph_msgs::msg::Clock::SharedPtr) { clockReceived = true; });
  while (!clockReceived) {
    rclcpp::spin_some(m_node);
  }
  subscription.reset();
}