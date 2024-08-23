#pragma once

#include "robotic_manipulation/scenario.h"

#include <vector>

class PutInBoxScenario : public Scenario {
public:
  PutInBoxScenario(bool flip = false, int seed = 0);
  ~PutInBoxScenario() override = default;

  std::string GetName() const override;
  std::string GetPrompt() const override;

  void Prepare(SceneController &scene, ArmController &arm) override;
  void CleanUp(SceneController &scene, ArmController &arm) override;

  void Play(ArmController &arm) override;

private:
  bool m_flip;
  int m_seed;
  std::vector<std::vector<double>> m_targets;

  double m_current_x = 0.40;
  double m_current_y = -0.339;
};