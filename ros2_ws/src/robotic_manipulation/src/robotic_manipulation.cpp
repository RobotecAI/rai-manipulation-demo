#include <memory>

#include "robotic_manipulation/arm_controller.h"
#include "robotic_manipulation/rosbag_recorder.h"
#include "robotic_manipulation/scenarios/move_toy.h"
#include "robotic_manipulation/scenarios/pickup.h"
#include "robotic_manipulation/scenarios/put_in_box.h"
#include "robotic_manipulation/scene_controller.h"
#include "robotic_manipulation/vla_controller.h"
#include <std_msgs/msg/float32_multi_array.hpp>

int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  // Create a SceneController
  auto sceneController = SceneController();
  auto armController = std::make_shared<ArmController>();

  std::vector<std::unique_ptr<Scenario>> scenarios;

  int first_scenario = 0;
  if (argc > 1) {
    first_scenario = std::stoi(argv[1]);
  }
  int last_scenario = 5;
  if (argc > 2) {
    last_scenario = std::stoi(argv[2]);
  }
  enum RunMode {
    VLA,
    RECORD,
    PLAY,
  };
  RunMode mode;
  if (argc > 3) {
    if (std::string(argv[3]) == "vla") {
      mode = VLA;
    } else if (std::string(argv[3]) == "record") {
      mode = RECORD;
    } else if (std::string(argv[3]) == "play") {
      mode = PLAY;
    }
  } else {
    mode = PLAY;
  }
  /*
  for (int i = first_scenario; i < last_scenario; i++) {
    scenarios.push_back(std::make_unique<MoveToyScenario>(false, i));
    scenarios.push_back(std::make_unique<MoveToyScenario>(true, i));
  }
  */
  /*
  for (int i = first_scenario; i < last_scenario; i++) {
    scenarios.push_back(std::make_unique<Pickup>(i));
  }
  */
  for (int i = first_scenario; i < last_scenario; i++) {
    scenarios.push_back(std::make_unique<PutInBoxScenario>(false, i));
  }

  armController->MoveThroughWaypoints({armController->CalculatePose(0.3, 0.0, 0.35)});

  std::vector<double> startingPose;
  startingPose = armController->CaptureJointValues();

  for (auto &scenario : scenarios) {
    armController->SetJointValues(startingPose);

    scenario->Prepare(sceneController, *armController);

    switch (mode) {
    case VLA: {
      VLAController vla;
      vla.Begin(*armController);
    } break;
    case RECORD: {
      RosbagRecorder recorder(scenario->GetName(), armController);
      recorder.BeginRecording();
      scenario->Play(*armController);
      recorder.EndRecording();
    } break;
    case PLAY:
      scenario->Play(*armController);
      break;
    }

    scenario->CleanUp(sceneController, *armController);
  }
  armController->SetJointValues(startingPose);

  // Shutdown ROS
  rclcpp::shutdown();

  return 0;
}