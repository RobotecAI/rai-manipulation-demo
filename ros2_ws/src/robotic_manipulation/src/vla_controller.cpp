#include "robotic_manipulation/vla_controller.h"

#include <std_msgs/msg/float32_multi_array.hpp>

VLAController::VLAController() {
  m_node = rclcpp::Node::make_shared("vla_controller");
  m_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  m_executor.add_node(m_node);
  m_spinner = std::thread([this]() { m_executor.spin(); });
}

VLAController::~VLAController() {
  m_executor.cancel();
  if (m_spinner.joinable()) {
    m_spinner.join();
  }
}

void VLAController::Begin(ArmController &arm) {
  auto logger = m_node->get_logger();

  auto current_pose = arm.GetEffectorPose();
  auto [current_x, current_y, current_z, current_rx, current_ry, current_rz] =
      std::make_tuple(current_pose[0], current_pose[1], current_pose[2],
                      current_pose[3], current_pose[4], current_pose[5]);

  RCLCPP_INFO(logger, "Current pose: %f %f %f %f %f %f", current_x, current_y,
              current_z, current_rx, current_ry, current_rz);

  auto action_subscription =
      m_node->create_subscription<std_msgs::msg::Float32MultiArray>(
          "/vla_action", 10,
          [&](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            std::string action = "";
            for (auto i : msg->data) {
              action += std::to_string(i) + " ";
            }
            RCLCPP_INFO(logger, "Action: %s", action.c_str());

            double dx = msg->data[0];
            double dy = msg->data[1];
            double dz = msg->data[2];

            double drx = msg->data[3];
            double dry = msg->data[4];
            double drz = msg->data[5];

            current_x += dx;
            current_y += dy;
            current_z += dz;

            current_rx += drx;
            current_ry += dry;
            current_rz -= drz;

            arm.MoveThroughWaypoints({arm.CalculatePose(
                current_x, current_y, current_z, current_rz)});

            if (msg->data[6] >= 0.5) {
              arm.Open();
            } else {
              arm.Close();
            }
          });

  std::this_thread::sleep_for(std::chrono::milliseconds(30000));
}