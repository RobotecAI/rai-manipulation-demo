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
      m_node->create_subscription<std_msgs::msg::Float32MultiArray>(
          "/object_state", 10,
          [&](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            std::string state = "";
            for (auto i : msg->data) {
              state += std::to_string(i) + " ";
            }
            RCLCPP_INFO(logger, "Requested state: %s", state.c_str());

            double dx = msg->data[0];
            double dy = msg->data[1];
            double dz = msg->data[2];
            double drx = msg->data[3];
            // double dry = msg->data[4];
            // double drz = msg->data[5];
            double gripper_state = msg->data[4];

            current_x = dx;
            current_y = dy;
            current_z = dz;
            current_gripper_state = gripper_state;;
            current_rx = drx;
            // current_ry = dry;
            // current_rz = drz;

            arm.MoveThroughWaypoints({arm.CalculatePose(
                current_x, current_y, current_z, current_rz)});

            if (msg->data[4] == 1.0) {
              arm.Open();
            } else {
              arm.Close();
            }

            arm.MoveThroughWaypoints({arm.CalculatePose(0.3, 0.0, 0.35)});
          });

  std::this_thread::sleep_for(std::chrono::milliseconds(300000));
}