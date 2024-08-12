#pragma once

#include <rclcpp/rclcpp.hpp>

#include "robotic_manipulation/arm_controller.h"

class RosbagRecorder {
public:
  RosbagRecorder(std::string const &name, std::shared_ptr<ArmController> arm);
  ~RosbagRecorder();

  void BeginRecording();
  void EndRecording();

private:
  std::string m_name;
  std::atomic_bool m_recording = false;
  std::thread m_recordingThread;
  std::shared_ptr<ArmController> m_arm;

  rclcpp::Node::SharedPtr m_node;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;
};