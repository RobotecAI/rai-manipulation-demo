#include "robotic_manipulation/scene_controller.h"

#include <Eigen/Geometry>

SceneController::SceneController() {
  m_node = rclcpp::Node::make_shared("scene_controller");
  m_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  m_executor.add_node(m_node);
  m_spinner = std::thread([this]() { m_executor.spin(); });

  auto logger = m_node->get_logger();

  m_spawnEntityClient =
      m_node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
  if (!m_spawnEntityClient->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(logger, "Service /spawn_entity not available");
  }

  m_deleteEntityClient =
      m_node->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
  if (!m_deleteEntityClient->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(logger, "Service /delete_entity not available");
  }

  m_detectionPublisher = m_node->create_publisher<vision_msgs::msg::Detection3DArray>(
      "/detections", rclcpp::QoS(10));
  
  m_detectionPublishTimer = m_node->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&SceneController::PublishDetections, this));
}

SceneController::~SceneController() {
  m_executor.cancel();
  if (m_spinner.joinable()) {
    m_spinner.join();
  }
}

void SceneController::SpawnToy(ToyType type, double x, double y, double z,
                               double r) {
  const double worldOffsetX = 1.593;
  const double worldOffsetY = 2.658;
  const double worldOffsetZ = 0.703;

  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  switch (type) {
  case Cube:
    request->name = "cube";
    break;
  case Triangle:
    request->name = "toy_triangle";
    break;
  case Cyllinder:
    request->name = "toy_cyllinder";
    break;
  case Box:
    request->name = "toy_box";
    break;
  case Apple:
    request->name = "apple";
    break;
  case Avocado:
    request->name = "avocado";
    break;
  case Bottle:
    request->name = "bottle";
    break;
  case Bread:
    request->name = "bread";
    break;
  case Broccoli:
    request->name = "broccoli";
    break;
  case Can:
    request->name = "can";
    break;
  case Carrot:
    request->name = "carrot";
    break;
  case Cheese:
    request->name = "cheese";
    break;
  case Corn:
    request->name = "corn";
    break;
  case Crossaint:
    request->name = "crossaint";
    break;
  case Cupcake:
    request->name = "cupcake";
    break;
  case Donut:
    request->name = "donut";
    break;
  case Foodcan:
    request->name = "foodcan";
    break;
  case Jam:
    request->name = "jam";
    break;
  case Lemon:
    request->name = "lemon";
    break;
  case Orange:
    request->name = "orange";
    break;
  case Pear:
    request->name = "pear";
    break;
  case Sponge:
    request->name = "sponge";
    break;
  case Tomato:
    request->name = "tomato";
    break;
  case Yogurt:
    request->name = "yogurt";
    break;
  case DebugCube:
    request->name = "debug_cube";
    break;
  }

  request->initial_pose.position.x = x + worldOffsetX;
  request->initial_pose.position.y = y + worldOffsetY;
  request->initial_pose.position.z = z + worldOffsetZ;

  auto rot = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitZ());
  auto quat = Eigen::Quaterniond(rot);
  request->initial_pose.orientation.x = quat.x();
  request->initial_pose.orientation.y = quat.y();
  request->initial_pose.orientation.z = quat.z();
  request->initial_pose.orientation.w = quat.w();

  auto result = m_spawnEntityClient->async_send_request(request);
  result.wait();
  auto status_message = result.get()->status_message;
  RCLCPP_INFO(m_node->get_logger(), "Spawned entity: %s",
              status_message.c_str());
  m_spawnedEntities.push_back(status_message);

  auto relativePose = request->initial_pose;
  relativePose.position.x -= worldOffsetX;
  relativePose.position.y -= worldOffsetY;
  relativePose.position.z -= worldOffsetZ;
  m_spawnedPoses[status_message] = relativePose;
  m_objectClasses[status_message] = request->name;
}

void SceneController::ClearToys() {
  for (auto const &entity : m_spawnedEntities) {
    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = entity;
    auto result = m_deleteEntityClient->async_send_request(request);
    result.wait();
  }
  m_spawnedEntities.clear();
}

void SceneController::PublishDetections() {
  auto detections = vision_msgs::msg::Detection3DArray();
  for (auto const &entity : m_spawnedEntities) {
    auto detection = vision_msgs::msg::Detection3D();
    detection.header.frame_id = "world";
    detection.header.stamp = m_node->now();
    
    auto object = vision_msgs::msg::ObjectHypothesisWithPose();
    object.hypothesis.class_id = "cube";
    object.hypothesis.score = 1.0;
    object.pose.pose = m_spawnedPoses[entity];

    detection.results.push_back(object);
    detection.id = entity;
    detection.bbox.size.x = 0.08;
    detection.bbox.size.y = 0.08;
    detection.bbox.size.z = 0.08;
    detection.bbox.center = m_spawnedPoses[entity];
    detections.detections.push_back(detection);
  }
  if (!detections.detections.empty())
    m_detectionPublisher->publish(detections);
}