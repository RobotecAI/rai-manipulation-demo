#include <memory>

#include "robotic_manipulation/arm_controller.h"
#include "robotic_manipulation/rosbag_recorder.h"
#include "robotic_manipulation/scenarios/move_toy.h"
#include "robotic_manipulation/scenarios/pickup.h"
#include "robotic_manipulation/scene_controller.h"
#include <std_msgs/msg/float32_multi_array.hpp>

int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "robotic_manipulation",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("robotic_manipulation");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // Create a SceneController
  auto sceneController = SceneController();
  auto armController = std::make_shared<ArmController>();

  std::vector<std::unique_ptr<Scenario>> scenarios;
  /*
  for (int i = 0; i < 50; i++) {
    scenarios.push_back(std::make_unique<MoveToyScenario>(false, i));
    scenarios.push_back(std::make_unique<MoveToyScenario>(true, i));
  }
  */
  for (int i = 188; i < 300; i++) {
    scenarios.push_back(std::make_unique<Pickup>(i));
  }

  auto vla_test = [&]() {
    auto current_pose = armController->GetEffectorPose();
    auto [current_x, current_y, current_z, current_rx, current_ry, current_rz] =
        std::make_tuple(current_pose[0], current_pose[1], current_pose[2],
                        current_pose[3], current_pose[4], current_pose[5]);
    RCLCPP_INFO(logger, "Current pose: %f %f %f %f %f %f", current_x, current_y,
                current_z, current_rx, current_ry, current_rz);

    auto action_subscription =
        node->create_subscription<std_msgs::msg::Float32MultiArray>(
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
              current_rz += drz;

              armController->MoveThroughWaypoints({armController->CalculatePose(
                  current_x, current_y, current_z, current_rz)});

              if (msg->data[6] >= 0.5) {
                armController->Open();
              } else {
                armController->Close();
              }
            });

    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  };

  for (auto &scenario : scenarios) {
    // auto &scenario = scenarios[0];
    scenario->Prepare(sceneController, *armController);

    RosbagRecorder recorder(scenario->GetName(), armController);
    recorder.BeginRecording();
    scenario->Play(*armController);
    recorder.EndRecording();

    scenario->CleanUp(sceneController, *armController);
  }

  executor.cancel();
  spinner.join();
  // Shutdown ROS
  rclcpp::shutdown();

  return 0;
}