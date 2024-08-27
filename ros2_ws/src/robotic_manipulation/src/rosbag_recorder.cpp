#include "robotic_manipulation/rosbag_recorder.h"

#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

RosbagRecorder::RosbagRecorder(std::string const &name,
                               std::shared_ptr<ArmController> arm)
    : m_name(name), m_arm(arm),
      m_node(std::make_shared<rclcpp::Node>(name + "_recorder")) {

  m_executor.add_node(m_node);
  m_spinner = std::thread([&]() { m_executor.spin(); });
}

RosbagRecorder::~RosbagRecorder() {
  m_executor.cancel();
  if (m_spinner.joinable()) {
    m_spinner.join();
  }
}

void RosbagRecorder::BeginRecording() {

  m_recording = true;
  m_recordingThread = std::thread([&]() {
    auto mutex = std::mutex();
    //auto latest_image = sensor_msgs::msg::Image::SharedPtr();
    auto image_topics = std::vector<std::string>{"/vla_image5"};
    auto latest_images = std::vector<sensor_msgs::msg::Image::SharedPtr>(image_topics.size());
    auto writer = std::make_unique<rosbag2_cpp::Writer>();

    auto image_subscriptions = std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>();
    for (std::size_t i = 0; i < image_topics.size(); i++) {
      image_subscriptions.push_back(
          m_node->create_subscription<sensor_msgs::msg::Image>(
              image_topics[i], 10, [&, i](sensor_msgs::msg::Image::SharedPtr msg) {
                mutex.lock();
                latest_images[i] = msg;
                mutex.unlock();
              }));
    }

    auto last_pose = m_arm->GetEffectorPose();
    auto last_images = std::vector<sensor_msgs::msg::Image::SharedPtr>(image_topics.size());

    writer.reset();
    writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(m_name);
    rosbag2_storage::TopicMetadata tm{};
    tm.name = "/vla_action";
    tm.type = "std_msgs/msg/Float32MultiArray";
    tm.serialization_format = rmw_get_serialization_format();
    writer->create_topic(tm);
    tm.name = "/vla_state";
    tm.type = "std_msgs/msg/Float32MultiArray";
    writer->create_topic(tm);
    for (std::size_t i = 0; i < image_topics.size(); i++) {
      tm.name = image_topics[i];
      tm.type = "sensor_msgs/msg/Image";
      writer->create_topic(tm);
    }

    RCLCPP_INFO(m_node->get_logger(), "Recording to %s", m_name.c_str());

    while (m_recording.load()) {
      auto pose = m_arm->GetEffectorPose();
      auto [dx, dy, dz, dR, dP, dY] =
          std::make_tuple(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
      dx -= last_pose[0];
      dy -= last_pose[1];
      dz -= last_pose[2];
      dR -= last_pose[3];
      dP -= last_pose[4];
      dY -= last_pose[5];
      if (last_images[0]) {
        auto time = m_node->get_clock()->now();
        {
          std_msgs::msg::Float32MultiArray msg;
          msg.data = {(float)dx,
                      (float)dy,
                      (float)dz,
                      (float)dR,
                      (float)dP,
                      (float)dY,
                      (float)m_arm->GetGripper()};
          // RCLCPP_INFO(m_node->get_logger(),
          //             "dXYZ: (%f, %f, %f), dRPY: (%f, %f, %f), gripper: %f",
          //             dx, dy, dz, dR, dP, dY, (float)m_arm->GetGripper());
          writer->write(msg, "/vla_action", time);

          auto state = std_msgs::msg::Float32MultiArray();
          state.data = {(float)last_pose[0],       (float)last_pose[1],
                        (float)last_pose[2],       (float)last_pose[3],
                        (float)last_pose[4],       (float)last_pose[5],
                        (float)m_arm->GetGripper()};
          writer->write(state, "/vla_state", time);
        }

        {
          mutex.lock();
          // image_publisher->publish(*last_image.get());
          for (std::size_t i = 0; i < image_topics.size(); i++) {
            if (latest_images[i]) {
              writer->write(*latest_images[i].get(), image_topics[i], time);
              last_images[i] = latest_images[i];
            }
          }
          mutex.unlock();
        }
        RCLCPP_INFO(m_node->get_logger(), "Image saved");
      } else {
        RCLCPP_INFO(m_node->get_logger(), "No image");
      }

      last_pose = pose;
      last_images = latest_images;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });
}

void RosbagRecorder::EndRecording() {
  m_recording = false;
  if (m_recordingThread.joinable()) {
    m_recordingThread.join();
  }
}