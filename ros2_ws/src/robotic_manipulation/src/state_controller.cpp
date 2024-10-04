#include "robotic_manipulation/state_controller.h"

#include <std_msgs/msg/float32_multi_array.hpp>

StateController::StateController() {
  m_node = rclcpp::Node::make_shared("state_controller");
  m_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  m_executor.add_node(m_node);
  m_spinner = std::thread([this]() { m_executor.spin(); });
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

  auto state_subscription =
      m_node->create_subscription<geometry_msgs::msg::Pose>(
          "/goal_state", 10, [&](geometry_msgs::msg::Pose::SharedPtr msg) {
            std::string state = "";
            for (auto i : {msg->position.x, msg->position.y, msg->position.z,
                           msg->orientation.x, msg->orientation.y,
                           msg->orientation.z, msg->orientation.w}) {
              state += std::to_string(i) + " ";
            }
            RCLCPP_INFO(logger, "Requested state: %s", state.c_str());

            arm.Open();
            {
              auto pose = *msg;
              pose.position.z += 0.1;
              arm.MoveThroughWaypoints({pose});
            }
            arm.MoveThroughWaypoints({*msg});
            arm.Close();
            msg->position.z += 0.3;
            arm.MoveThroughWaypoints({*msg});

            const double box_x = 0.40;
            const double box_y = -0.339;
            arm.MoveThroughWaypoints({arm.CalculatePose(box_x, box_y, 0.35)});
            arm.Open();
            arm.MoveThroughWaypoints(
                {arm.CalculatePose(0.3, 0.0, 0.35)});
          });

  std::this_thread::sleep_for(std::chrono::milliseconds(300000));
}