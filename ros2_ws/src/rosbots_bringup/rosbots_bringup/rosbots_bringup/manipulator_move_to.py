from gazebo_msgs.srv import SpawnEntity

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from threading import Thread
from rai_interfaces.srv import ManipulatorMoveTo
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

class ManipulationClient(Node):
    def __init__(self):
        super().__init__('navigator')

        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('position', [0.0, 0.0, 0.0])
        
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.position = self.get_parameter('position').get_parameter_value().double_array_value

        self.client = self.create_client(
            ManipulatorMoveTo,
            "/manipulator_move_to",
        )
        self.get_logger().info(f"Waiting for service {self.client.srv_name}")
        self.client.wait_for_service()

        # constant quaternion
        self.quaternion = Quaternion(x=0.9238795325112867, y=-0.3826834323650898, z=0.0, w=0.0)

    def send_request(self):
        x = self.position[0]
        y = self.position[1]
        z = self.position[2]
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'world'
        pose_stamped.pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=self.quaternion,
        )

        # if task == "drop":
        #     pose_stamped.pose.position.z += self.additional_height

        # pose_stamped.pose.position.z = np.max(
        #     [pose_stamped.pose.position.z, self.min_z]
        # )

        request = ManipulatorMoveTo.Request()
        request.target_pose = pose_stamped

        request.initial_gripper_state = True  # open
        request.final_gripper_state = False  # closed
        
        # if task == "grab":
        #     request.initial_gripper_state = True  # open
        #     request.final_gripper_state = False  # closed
        # else:
        #     request.initial_gripper_state = False  # closed
        #     request.final_gripper_state = True  # open

        future = self.client.call_async(request)
        self.get_logger().debug(
            f"Calling ManipulatorMoveTo service with request: x={request.target_pose.pose.position.x:.2f}, y={request.target_pose.pose.position.y:.2f}, z={request.target_pose.pose.position.z:.2f}"
        )
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().info(f"Service call failed for point ({x:.2f}, {y:.2f}, {z:.2f}).")
            return

        if response.success:
            self.get_logger().info(f"End effector successfully positioned at coordinates ({x:.2f}, {y:.2f}, {z:.2f}). Note: The status of object interaction (grab/drop) is not confirmed by this movement.")
        else:
            self.get_logger().info(f"Failed to position end effector at coordinates ({x:.2f}, {y:.2f}, {z:.2f}).")

def main(args=None):
    rclpy.init(args=args)

    client = ManipulationClient()
    client.send_request()
    client.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()