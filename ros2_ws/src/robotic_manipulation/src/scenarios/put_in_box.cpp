#include "robotic_manipulation/scenarios/put_in_box.h"

#include <random>

PutInBoxScenario::PutInBoxScenario(bool flip, int seed)
    : m_flip(flip), m_seed(seed) {}

std::string PutInBoxScenario::GetName() const {
  return "put_in_box_" + std::to_string(m_seed);
}

std::string PutInBoxScenario::GetPrompt() const { return "put block in box"; }

void PutInBoxScenario::Prepare(SceneController &scene, ArmController &arm) {
  srand(m_seed);

  scene.SpawnToy(ToyType::Box, m_current_x, m_current_y, 0.1, 0.0);

  double x_min = 0.3;
  double x_max = 0.5;
  double y_min = 0.15;
  double y_max = 0.4;

  double x_ = static_cast<double>(rand()) / RAND_MAX * (x_max - x_min) + x_min;
  double y_ = static_cast<double>(rand()) / RAND_MAX * (y_max - y_min) + y_min;

  double a = static_cast<double>(rand()) / RAND_MAX * M_PI / 2.0 - M_PI / 4.0;
  int toy = 0;
  m_targets.push_back({x_, y_, a});
  scene.SpawnToy((ToyType)toy, x_, y_, 0.1, -a);

  arm.MoveThroughWaypoints({arm.CalculatePose(0.3, 0.0, 0.35)});
  arm.Open();

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

void PutInBoxScenario::CleanUp(SceneController &scene, ArmController &) {
  scene.ClearToys();
}

void PutInBoxScenario::Play(ArmController &arm) {
  const double high_z = 0.22;
  const double low_z = 0.14;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  for (auto target : m_targets) {
    auto [x, y, a] = std::make_tuple(target[0], target[1], target[2]);

    // move to the target
    waypoints.push_back(arm.CalculatePose(x, y, high_z));
    waypoints.push_back(arm.CalculatePose(x, y, low_z, a));
    arm.MoveThroughWaypoints(waypoints);
    waypoints.clear();

    // pick up the target
    arm.Close();

    // move to the final position
    arm.MoveThroughWaypoints(
        {arm.CalculatePose(x, y, 0.35, 0.0),
         arm.CalculatePose(m_current_x, m_current_y, 0.35, 0.0),
         arm.CalculatePose(m_current_x, m_current_y, 0.2, 0.0)});

    // release the target
    arm.Open();

    waypoints.push_back(arm.CalculatePose(m_current_x, m_current_y, 0.35));

    if (m_flip) {
      m_current_y -= 0.1;
    } else {
      m_current_y += 0.1;
    }
  }

  arm.MoveThroughWaypoints(waypoints);

  arm.MoveThroughWaypoints({arm.CalculatePose(0.3, 0.0, 0.35)});
}