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

#include "robotic_manipulation/rai_manipulation_interface_node.h"

#include <std_msgs/msg/float32_multi_array.hpp>

RaiManipulationInterfaceNode::RaiManipulationInterfaceNode()
    : m_node(rclcpp::Node::make_shared("state_controller")) {
  m_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
}

void RaiManipulationInterfaceNode::Initialize(ArmController &arm) {
  auto logger = m_node->get_logger();

  auto currentPose = arm.GetEffectorPose();
  auto [currentX, currentY, currentZ, currentRX, currentRY, currentRZ] =
      std::make_tuple(currentPose[0], currentPose[1], currentPose[2],
                      currentPose[3], currentPose[4], currentPose[5]);

  auto currentGripperState = arm.GetGripper();

  m_startingPose = arm.CaptureJointValues();

  RCLCPP_INFO(logger, "Current pose: %f %f %f %f %f %f", currentX, currentY,
              currentRZ, currentRX, currentRY, currentRZ);
  RCLCPP_INFO(logger, "Current gripper state: %d", currentGripperState);

  m_moveToService = m_node->create_service<
      rai_interfaces::srv::ManipulatorMoveTo>(
      "/manipulator_move_to",
      [&](std::shared_ptr<rai_interfaces::srv::ManipulatorMoveTo::Request> const
              request,
          std::shared_ptr<rai_interfaces::srv::ManipulatorMoveTo::Response>
              response) {
        RCLCPP_INFO(logger, "Received move request");

        response->success = false;

        arm.SetReferenceFrame(request->target_pose.header.frame_id);
        RCLCPP_INFO(logger, "Set reference frame to: %s",
                    request->target_pose.header.frame_id.c_str());

        // Print current pose
        auto currentPose = arm.GetEffectorPose();
        auto [currentX, currentY, currentZ, currentRX, currentRY, currentRZ] =
            std::make_tuple(currentPose[0], currentPose[1], currentPose[2],
                            currentPose[3], currentPose[4], currentPose[5]);

        RCLCPP_INFO(logger, "Current pose: %f %f %f %f %f %f", currentX,
                    currentY, currentRZ, currentRX, currentRY, currentRZ);
        RCLCPP_INFO(logger, "Target pose: %f %f %f %f %f %f",
                    request->target_pose.pose.position.x,
                    request->target_pose.pose.position.y,
                    request->target_pose.pose.position.z,
                    request->target_pose.pose.orientation.x,
                    request->target_pose.pose.orientation.y,
                    request->target_pose.pose.orientation.z);

        if (request->initial_gripper_state) {
          arm.Open();
        } else {
          arm.Close();
        }

        {
          using std::min;
          using std::max;
          auto currentPose = arm.GetEffectorPose();
          auto aboveCurrent = currentPose;
          auto calculateZAboveTarget = [&](double targetZ) {
            double constexpr MinimumZ = 0.3;
            double constexpr MaximumZ = 0.4;
            double constexpr ZOffset = 0.1;

            return min(MaximumZ, max(targetZ + ZOffset, MinimumZ));
          };
          aboveCurrent[2] = calculateZAboveTarget(aboveCurrent[2]);
          auto aboveTarget = request->target_pose.pose;

          aboveTarget.position.z =
              calculateZAboveTarget(aboveTarget.position.z);
          if (!arm.MoveThroughWaypoints(
                  {arm.CalculatePose(aboveCurrent[0], aboveCurrent[1],
                                     aboveCurrent[2]),
                   aboveTarget, request->target_pose.pose}))
            return;

          if (request->final_gripper_state) {
            arm.Open();
          } else {
            arm.Close();
          }
        }

        if (!arm.MoveThroughWaypoints({request->target_pose.pose}))
          return;

        response->success = true;
      });

  RCLCPP_INFO(logger, "Service /manipulator_move_to is ready");

  m_resetService = m_node->create_service<std_srvs::srv::Trigger>(
      "/reset_manipulator",
      [&]([[maybe_unused]] std::shared_ptr<
              std_srvs::srv::Trigger::Request> const request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(logger, "Received reset request");
        response->success = arm.SetJointValues(m_startingPose);
      });
}

void RaiManipulationInterfaceNode::Spin() {
  m_executor.add_node(m_node);
  m_executor.spin();
}