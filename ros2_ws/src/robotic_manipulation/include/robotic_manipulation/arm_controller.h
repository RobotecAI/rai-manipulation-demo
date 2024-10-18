#pragma once

#include <moveit/move_group_interface/move_group_interface.h>

class ArmController {
public:
  ArmController();
  ~ArmController();

  geometry_msgs::msg::Pose CalculatePose(double x, double y, double z,
                                         double r = 0.0);

  bool MoveThroughWaypoints(const std::vector<geometry_msgs::msg::Pose>& waypoints);

  void Open();
  void Close();

  std::vector<double> GetEffectorPose();
  bool GetGripper();

  std::vector<double> CaptureJointValues();
  void SetJointValues(std::vector<double> const &jointValues);

  void SetReferenceFrame(std::string const &frame);

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_pandaArm;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_hand;

  std::atomic_bool gripper = false;

  rclcpp::Node::SharedPtr m_node;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;
};
