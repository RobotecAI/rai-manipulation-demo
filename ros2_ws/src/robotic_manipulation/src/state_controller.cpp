#include "robotic_manipulation/state_controller.h"

#include <std_msgs/msg/float32_multi_array.hpp>
#include "rai_interfaces/srv/manipulator_move_to.hpp"


StateController::StateController() 
  : m_node(rclcpp::Node::make_shared("state_controller"))
{
  m_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
}

StateController::~StateController() {
  m_executor.cancel();
  if (m_spinner.joinable()) {
    m_spinner.join();
  }
}

void StateController::Begin(ArmController &arm) {
  auto logger = m_node->get_logger();

  auto current_pose = arm.GetEffectorPose();
  auto [current_x, current_y, current_z, current_rx, current_ry, current_rz] =
      std::make_tuple(current_pose[0], current_pose[1], current_pose[2],
                      current_pose[3], current_pose[4], current_pose[5]);

  auto current_gripper_state = arm.GetGripper();

  RCLCPP_INFO(logger, "Current pose: %f %f %f %f %f %f", current_x, current_y,
              current_z, current_rx, current_ry, current_rz);
  RCLCPP_INFO(logger, "Current gripper state: %d", current_gripper_state);

  auto service = m_node->create_service<rai_interfaces::srv::ManipulatorMoveTo>(
    "/manipulator_move_to",
    [&](const std::shared_ptr<rai_interfaces::srv::ManipulatorMoveTo::Request> request,
        std::shared_ptr<rai_interfaces::srv::ManipulatorMoveTo::Response> response) {
      RCLCPP_INFO(logger, "Received move request");

      response->success = false;

      arm.SetReferenceFrame(request->target_pose.header.frame_id);

      if (request->initial_gripper_state) {
        arm.Open();
      } else {
        arm.Close();
      }

      {
        auto current_pose = arm.GetEffectorPose();
        auto above_current = current_pose;
        above_current[2] = 0.35;
        auto above_target = request->target_pose.pose;
        above_target.position.z = 0.35;
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
    }
  );

  RCLCPP_INFO(logger, "Service /manipulator_move_to is ready");

  // Add node to executor and spin in the main thread
  m_executor.add_node(m_node);
  m_executor.spin();
}