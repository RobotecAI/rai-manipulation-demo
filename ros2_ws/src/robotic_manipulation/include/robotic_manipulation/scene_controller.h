#pragma once

#include <gazebo_msgs/srv/delete_entity.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <vision_msgs/vision_msgs/msg/detection3_d_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

enum ToyType { Cube = 0, Triangle = 1, Cyllinder = 2, Box = 3 };

class SceneController {
public:
  SceneController();
  ~SceneController();

  void SpawnToy(ToyType type, double x, double y, double z, double r = 0.0);
  void ClearToys();

private:
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr m_spawnEntityClient;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr
      m_deleteEntityClient;

  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr
      m_detectionPublisher;

  rclcpp::TimerBase::SharedPtr m_detectionPublishTimer;

  std::vector<std::string> m_spawnedEntities;
  std::unordered_map<std::string, geometry_msgs::msg::Pose> m_spawnedPoses;
  std::unordered_map<std::string, std::string> m_objectClasses;

  rclcpp::Node::SharedPtr m_node;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;

  void PublishDetections();
};