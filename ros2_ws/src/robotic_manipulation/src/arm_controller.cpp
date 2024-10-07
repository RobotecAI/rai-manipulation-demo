#include "robotic_manipulation/arm_controller.h"

ArmController::ArmController() {
  m_node = rclcpp::Node::make_shared("arm_controller");
  m_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  m_executor.add_node(m_node);
  m_spinner = std::thread([this]() { m_executor.spin(); });

  m_pandaArm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      m_node, "panda_arm");
  m_hand = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      m_node, "hand");

  m_pandaArm->setMaxVelocityScalingFactor(1.0);
  m_pandaArm->setMaxAccelerationScalingFactor(1.0);
}

ArmController::~ArmController() {
  m_executor.cancel();
  if (m_spinner.joinable()) {
    m_spinner.join();
  }
}

geometry_msgs::msg::Pose ArmController::CalculatePose(double x, double y,
                                                      double z, double r) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  auto quat = tf2::Quaternion();
  quat.setEuler(0.0, M_PI, tf2Radians(45.0) + r);
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();

  return pose;
}

bool ArmController::MoveThroughWaypoints(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
  auto logger = m_node->get_logger();
  moveit_msgs::msg::RobotTrajectory trajectory;
  if (m_pandaArm->computeCartesianPath(waypoints, 0.01, 0.0, trajectory) ==
      -1) {
    RCLCPP_ERROR(logger,
                 "MoveThroughWaypoints: Failed to compute Cartesian path");
    return false;
  }

  while (m_pandaArm->execute(trajectory) !=
         moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "MoveThroughWaypoints: Failed to execute trajectory");
    return false;
  }
  
  return true;
}

void ArmController::Open() {
  gripper.store(true);
  m_hand->setNamedTarget("open");
  while (m_hand->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(m_node->get_logger(), "Failed to open hand");
  }
}

void ArmController::Close() {
  gripper.store(false);
  m_hand->setNamedTarget("close");
  while (m_hand->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(m_node->get_logger(), "Failed to close hand");
  }
}

std::vector<double> ArmController::GetEffectorPose() {
  auto pose = m_pandaArm->getCurrentPose().pose;
  auto rotation = tf2::Quaternion(pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3 m(rotation);
  m.getRPY(pose.orientation.x, pose.orientation.y, pose.orientation.z, 0);
  return {pose.position.x,           pose.position.y,
          pose.position.z,           pose.orientation.x,
          pose.orientation.y - M_PI, pose.orientation.z - tf2Radians(135.0)};
};

bool ArmController::GetGripper() { return gripper.load(); }

std::vector<double> ArmController::CaptureJointValues() {
  return m_pandaArm->getCurrentJointValues();
}

void ArmController::SetJointValues(std::vector<double> const &jointValues) {
  m_pandaArm->setJointValueTarget(jointValues);
  if (m_pandaArm->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(m_node->get_logger(), "Failed to set joint values");
  }
}

void ArmController::SetReferenceFrame(std::string const &frame) {
  m_pandaArm->setPoseReferenceFrame(frame);
}