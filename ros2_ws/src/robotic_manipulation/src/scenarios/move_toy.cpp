#include "robotic_manipulation/scenarios/move_toy.h"

#include <random>

MoveToyScenario::MoveToyScenario(bool flip, int seed)
    : m_flip(flip), m_seed(seed) {}

std::string MoveToyScenario::GetName() const {
  if (m_flip) {
    return "right_to_left_" + std::to_string(m_seed);
  } else {
    return "left_to_right_" + std::to_string(m_seed);
  }
}

std::string MoveToyScenario::GetPrompt() const {
  if (m_flip) {
    return "Move the black cube to the left side of the table.";
  } else {
    return "Move the black cube to the right side of the table.";
  }
}

void MoveToyScenario::Prepare(SceneController &scene, ArmController &arm) {
  srand(m_seed);

  double x_min = 0.3;
  double x_max = 0.5;
  double y_min = 0.15;
  double y_max = 0.4;

  if (m_flip) {
    y_min *= -1.0;
    y_max *= -1.0;
    std::swap(y_min, y_max);
    m_current_y *= -1.0;
  }

  // spawn 4 toys, one of them being the target and the rest distractors
  int i = 0;
  int target_i = rand() % 4;
  for (double x = x_min; x <= x_max; x += 0.15) {
    for (double y = y_min; y <= y_max; y += 0.15) {
      double x_ = x + static_cast<double>(rand()) / RAND_MAX * 0.1;
      double y_ = y + static_cast<double>(rand()) / RAND_MAX * 0.1;
      double a =
          static_cast<double>(rand()) / RAND_MAX * M_PI / 2.0 - M_PI / 4.0;
      int toy = 1 + (rand() % 2);
      if (i == target_i) {
        m_targets.push_back({x_, y_, a});
        toy = 0;
      }
      scene.SpawnToy((ToyType)toy, x_, y_, 0.1, -a);
      i++;
    }
  }

  arm.MoveThroughWaypoints({arm.CalculatePose(0.3, 0.0, 0.35)});
  arm.Open();

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

void MoveToyScenario::CleanUp(SceneController &scene, ArmController &) {
  scene.ClearToys();
}

void MoveToyScenario::Play(ArmController &arm) {
  const double high_z = 0.30;
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
        {arm.CalculatePose(x, y, high_z, 0.0),
         arm.CalculatePose(m_current_x, m_current_y, high_z, 0.0),
         arm.CalculatePose(m_current_x, m_current_y, low_z, 0.0)});

    // release the target
    arm.Open();

    waypoints.push_back(arm.CalculatePose(m_current_x, m_current_y, high_z));

    if (m_flip) {
      m_current_y -= 0.1;
    } else {
      m_current_y += 0.1;
    }
  }

  arm.MoveThroughWaypoints(waypoints);
}