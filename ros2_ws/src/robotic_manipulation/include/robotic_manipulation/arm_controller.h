#pragma once

#include <moveit/move_group_interface/move_group_interface.h>

class ArmController {
public:
  ArmController() = default;
  ~ArmController();

  void Initialize();

  geometry_msgs::msg::Pose CalculatePose(double x, double y, double z,
                                         double r = 0.0);

  bool
  MoveThroughWaypoints(std::vector<geometry_msgs::msg::Pose> const &waypoints);

  void Open();
  void Close();

  std::vector<double> GetEffectorPose();
  bool GetGripper();

  std::vector<double> CaptureJointValues();
  bool SetJointValues(std::vector<double> const &jointValues);

  void SetReferenceFrame(std::string const &frame);

private:
  void WaitForClockMessage();

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_pandaArm;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_hand;

  std::atomic_bool gripper = false;

  rclcpp::Node::SharedPtr m_node;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;
};
