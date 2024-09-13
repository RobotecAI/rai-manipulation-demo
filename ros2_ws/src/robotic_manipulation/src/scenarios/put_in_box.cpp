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

  scene.SpawnToy(ToyType::Box, m_box_x, m_box_y, 0.1, 0.0);

  using Point = geometry_msgs::msg::Point;
  auto cube_positions = std::vector<Point>();

  const double x_min = 0.3;
  const double x_max = 0.51;
  const double y_min = 0.1;
  const double y_max = 0.41;

  for (double x = x_min; x <= x_max; x += 0.05) {
    for (double y = y_min; y <= y_max; y += 0.05) {
      Point p;
      p.x = x;
      p.y = y;
      p.z = 0.1;
      for (double a = -M_PI / 4.0; a < M_PI / 4.0; a += M_PI / 8.0) {
        p.z = a;
        cube_positions.push_back(p);
      }
    }
  }

  if (m_seed / 8 >= (int) cube_positions.size()) {
    return;
  }

  auto cube_position = cube_positions[m_seed / 8];

  auto start_positions = std::vector<Point>();

  {
    Point p;
    p.x = 0.3;
    p.y = 0.0;
    p.z = 0.35;
    start_positions.push_back(p);

    p.x = 0.5;
    p.y = 0.0;
    start_positions.push_back(p);

    p.x = 0.5;
    p.y = 0.5;
    start_positions.push_back(p);

    p.x = 0.3;
    p.y = 0.5;
    start_positions.push_back(p);
  }

  for (int i = 0; i < 4; i++) {
    Point p;
    p.x = 0; p.y = 0; p.z = 0.2;
    if (i == 0) {
      p.x = cube_position.x + 0.1;
      p.y = cube_position.y;
    } else if (i == 1) {
      p.x = cube_position.x;
      p.y = cube_position.y + 0.1;
    } else if (i == 2) {
      p.x = cube_position.x - 0.1;
      p.y = cube_position.y;
    } else if (i == 3) {
      p.x = cube_position.x;
      p.y = cube_position.y - 0.1;
    }
    start_positions.push_back(p);
  }

  auto start_position = start_positions[m_seed % 8];

  scene.SpawnToy(ToyType::Cube, cube_position.x, cube_position.y, 0.1, 0.1);
  scene.SpawnToy(ToyType::Cube, cube_position.x, cube_position.y + 0.3, 0.1, 0.0);
  scene.SpawnToy(ToyType::Cube, cube_position.x, cube_position.y - 0.2, 0.1, 1.0);

  m_targets.push_back({cube_position.x, cube_position.y, -cube_position.z});

  arm.MoveThroughWaypoints({arm.CalculatePose(start_position.x, start_position.y, start_position.z)});
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
    waypoints.push_back(arm.CalculatePose(x, y, high_z, a));
    waypoints.push_back(arm.CalculatePose(x, y, low_z, a));
    arm.MoveThroughWaypoints(waypoints);
    waypoints.clear();

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // pick up the target
    arm.Close();

    // move to the final position
    arm.MoveThroughWaypoints(
        {arm.CalculatePose(x, y, 0.35, 0.0),
         arm.CalculatePose(m_box_x, m_box_y, 0.35, 0.0),
         arm.CalculatePose(m_box_x, m_box_y, 0.2, 0.0)});

    // release the target
    arm.Open();

    waypoints.push_back(arm.CalculatePose(m_box_x, m_box_y, 0.35));
  }

  arm.MoveThroughWaypoints(waypoints);

  arm.MoveThroughWaypoints({arm.CalculatePose(0.3, 0.0, 0.35)});
}