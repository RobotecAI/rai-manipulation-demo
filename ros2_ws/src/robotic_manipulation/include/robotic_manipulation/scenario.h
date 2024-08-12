#pragma once

#include "robotic_manipulation/arm_controller.h"
#include "robotic_manipulation/scene_controller.h"

class Scenario {
public:
  virtual ~Scenario() = default;

  virtual std::string GetName() const = 0;
  virtual std::string GetPrompt() const = 0;

  virtual void Prepare(SceneController &scene, ArmController &arm);
  virtual void CleanUp(SceneController &scene, ArmController &arm);

  virtual void Play(ArmController &arm) = 0;

  void BeginRecording();
  void EndRecording();

protected:
  bool m_recording = false;
  std::thread m_recordingThread;
};