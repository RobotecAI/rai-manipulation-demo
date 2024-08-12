#pragma once

#include "robotic_manipulation/scenario.h"

#include <vector>

class Pickup : public Scenario {
public:
  Pickup(int seed = 0);
  ~Pickup() override = default;

  std::string GetName() const override;
  std::string GetPrompt() const override;

  void Prepare(SceneController &scene, ArmController &arm) override;
  void CleanUp(SceneController &scene, ArmController &arm) override;

  void Play(ArmController &arm) override;

private:
  int m_seed;
  std::vector<double> m_target;
};