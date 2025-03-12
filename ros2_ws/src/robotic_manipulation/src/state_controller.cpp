#include "robotic_manipulation/state_controller.h"

#include <std_msgs/msg/float32_multi_array.hpp>
#include "rai_interfaces/srv/manipulator_move_to.hpp"
#include "rai_interfaces/srv/manipulate_object.hpp"

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

  auto old_service_function = [&] (geometry_msgs::msg::PoseStamped target_pose, bool initial_gripper_state, bool final_gripper_state)
  {
    RCLCPP_INFO(logger, "Received move request");

    arm.SetReferenceFrame(target_pose.header.frame_id);
    RCLCPP_INFO(logger, "Set reference frame to: %s", target_pose.header.frame_id.c_str());

    // Print current pose
    auto current_pose = arm.GetEffectorPose();
    auto [current_x, current_y, current_z, current_rx, current_ry, current_rz] =
        std::make_tuple(current_pose[0], current_pose[1], current_pose[2],
                        current_pose[3], current_pose[4], current_pose[5]);

    RCLCPP_INFO(logger, "Current pose: %f %f %f %f %f %f", current_x, current_y,
                current_z, current_rx, current_ry, current_rz);
    RCLCPP_INFO(logger, "Target pose: %f %f %f %f %f %f", 
                target_pose.pose.position.x, 
                target_pose.pose.position.y, 
                target_pose.pose.position.z,
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z);

    if (initial_gripper_state) {
      if (!arm.Open()) return false;
    } else {
      if (!arm.Close()) return false;
    }

    {
      using std::min;
      using std::max;
      auto current_pose = arm.GetEffectorPose();
      auto above_current = current_pose;
      above_current[2] = min(0.4, max(above_current[2] + 0.1, 0.3));
      auto above_target = target_pose.pose;

      above_target.position.z = min(0.4, max(above_target.position.z + 0.1, 0.3));
      if (!arm.MoveThroughWaypoints(
              {arm.CalculatePose(above_current[0], above_current[1],
                                  above_current[2]),
                above_target, target_pose.pose}))
        return false;

      if (final_gripper_state) {
        if (!arm.Open()) return false;
      } else {
        if (!arm.Close()) return false;
      }
    }

    if (!arm.MoveThroughWaypoints({target_pose.pose}))
      return false;

    return true;
  };

  auto service = m_node->create_service<rai_interfaces::srv::ManipulatorMoveTo>(
    "/manipulator_move_to",
    [&](const std::shared_ptr<rai_interfaces::srv::ManipulatorMoveTo::Request> request,
        std::shared_ptr<rai_interfaces::srv::ManipulatorMoveTo::Response> response) {
      response->success = old_service_function(request->target_pose, request->initial_gripper_state, request->final_gripper_state);
    }
  );

  auto service2 = m_node->create_service<rai_interfaces::srv::ManipulateObject>(
    "/manipulate_object",
    [&](const std::shared_ptr<rai_interfaces::srv::ManipulateObject::Request> request,
        std::shared_ptr<rai_interfaces::srv::ManipulateObject::Response> response) {
      response->success = old_service_function(request->grab_pose, true, false);
      if (response->success) {
        response->success = old_service_function(request->drop_pose, false, true);
      }
    }
  );

  RCLCPP_INFO(logger, "Service /manipulator_move_to is ready");

  // Add node to executor and spin in the main thread
  m_executor.add_node(m_node);
  m_executor.spin();
}