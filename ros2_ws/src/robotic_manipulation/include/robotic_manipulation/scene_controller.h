#pragma once

#include <gazebo_msgs/srv/delete_entity.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
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

  std::vector<std::string> m_spawnedEntities;

  rclcpp::Node::SharedPtr m_node;
  rclcpp::executors::SingleThreadedExecutor m_executor;
  std::thread m_spinner;
};