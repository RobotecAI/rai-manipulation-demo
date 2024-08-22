#include "robotic_manipulation/scenarios/pickup.h"

#include <random>

Pickup::Pickup(int seed) : m_seed(seed) {}

std::string Pickup::GetName() const {
  return "pickup_" + std::to_string(m_seed);
}

std::string Pickup::GetPrompt() const { return "pickup the block"; }

const double starting_height = 0.35;
//const double starting_height = 0.25;

void Pickup::Prepare(SceneController &scene, ArmController &arm) {
  srand(m_seed);

  double x_ = 0.4 + static_cast<double>(rand()) / RAND_MAX * 0.2 - 0.1;
  double y_ = static_cast<double>(rand()) / RAND_MAX * 0.6 - 0.3;
  double a = static_cast<double>(rand()) / RAND_MAX * M_PI / 2.0 - M_PI / 4.0;
  int toy = 0;
  m_target = {x_, y_, a};
  scene.SpawnToy((ToyType)toy, x_, y_, 0.1, -a);

  arm.MoveThroughWaypoints({arm.CalculatePose(0.3, 0.0, starting_height)});
  arm.Open();

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

void Pickup::CleanUp(SceneController &scene, ArmController &) {
  scene.ClearToys();
}

void Pickup::Play(ArmController &arm) {
  const double high_z = 0.19;
  const double low_z = 0.14;

  std::vector<geometry_msgs::msg::Pose> waypoints;

  auto [x, y, a] = std::make_tuple(m_target[0], m_target[1], m_target[2]);

  // move to the target
  waypoints.push_back(arm.CalculatePose(x, y, high_z, a));
  waypoints.push_back(arm.CalculatePose(x, y, low_z, a));
  arm.MoveThroughWaypoints(waypoints);
  waypoints.clear();

  // pick up the target
  arm.Close();

  // move to the final position
  arm.MoveThroughWaypoints({arm.CalculatePose(x, y, high_z, a),
                            arm.CalculatePose(0.3, 0.0, starting_height)});

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // release the target
  // arm.Open();
}