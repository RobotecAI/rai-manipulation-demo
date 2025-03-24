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

  auto current_pose = arm.GetEffectorPose();
  auto [current_x, current_y, current_z, current_rx, current_ry, current_rz] =
      std::make_tuple(current_pose[0], current_pose[1], current_pose[2],
                      current_pose[3], current_pose[4], current_pose[5]);

  auto current_gripper_state = arm.GetGripper();

  m_startingPose = arm.CaptureJointValues();

  RCLCPP_INFO(logger, "Current pose: %f %f %f %f %f %f", current_x, current_y,
              current_z, current_rx, current_ry, current_rz);
  RCLCPP_INFO(logger, "Current gripper state: %d", current_gripper_state);

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
        auto current_pose = arm.GetEffectorPose();
        auto [current_x, current_y, current_z, current_rx, current_ry,
              current_rz] =
            std::make_tuple(current_pose[0], current_pose[1], current_pose[2],
                            current_pose[3], current_pose[4], current_pose[5]);

        RCLCPP_INFO(logger, "Current pose: %f %f %f %f %f %f", current_x,
                    current_y, current_z, current_rx, current_ry, current_rz);
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
          auto current_pose = arm.GetEffectorPose();
          auto above_current = current_pose;
          auto calculate_z_above_target = [&](double target_z) {
            double const minimum_z = 0.3;
            double const maximum_z = 0.4;
            double const z_offset = 0.1;

            return min(maximum_z, max(target_z + z_offset, minimum_z));
          };
          above_current[2] = calculate_z_above_target(above_current[2]);
          auto above_target = request->target_pose.pose;

          above_target.position.z =
              calculate_z_above_target(above_target.position.z);
          if (!arm.MoveThroughWaypoints(
                  {arm.CalculatePose(above_current[0], above_current[1],
                                     above_current[2]),
                   above_target, request->target_pose.pose}))
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