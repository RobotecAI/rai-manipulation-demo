import rclpy
from rclpy.node import Node
from rclpy.client import Client
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from moveit import moveit
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from rai_interfaces.srv import ManipulatorMoveTo

class ScenarioManager(Node):
    def __init__(self, scenario_type):
        super().__init__('scenario_manager')
        self.scenario_type = scenario_type

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.tf2_buffer = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer, self)

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.scenario = None
    
    def timer_callback(self):
        if self.scenario == None:
            self.scenario = self.scenario_type(self.spawn_client, self.delete_client, self)
        self.scenario._pre_step()
        progress, terminated = self.scenario.step()
        self.get_logger().info(f'Task progress: {progress}')
        if terminated:
            self.get_logger().info('Scenario terminated')
            self.scenario = None

class Scenario:
    def __init__(self, spawn_client: Client, delete_client: Client, node: ScenarioManager):
        self.spawn_client = spawn_client
        self.delete_client = delete_client
        self.node = node
        self.entities = {}
        self.steps = 0

        self.reset()

    def __del__(self):
        for name in self.entities:
            self.delete_entity(name)
    
    def reset(self):
        self.steps = 0
        for name in self.entities:
            self.delete_entity(name)
        self.entities = {}
        
    def spawn_entity(self, prefab_name: str, name: str, pose: Pose):
        req = SpawnEntity.Request()
        req.name = prefab_name
        req.xml = ''
        req.robot_namespace = name
        req.initial_pose = pose

        self.spawn_client.call_async(req).add_done_callback(lambda future: self.entity_spawned_callback(future, name))

    def delete_entity(self, name: str):
        req = DeleteEntity.Request()
        req.name = self.entities[name]

        self.delete_client.call_async(req)
    
    def get_entity_pose(self, name: str):
        pose = PoseStamped()
        entity_frame = name + '/'
        pose.header.frame_id = entity_frame
        pose = self.node.tf2_buffer.transform(pose, entity_frame + 'odom', timeout=rclpy.time.Duration(seconds=5.0))
        return pose.pose
    
    def _pre_step(self):
        self.steps += 1

    def step(self):
        return 0.0, False

    def entity_spawned_callback(self, future: Future, name: str):
        result = future.result()
        if result.success:
            self.node.get_logger().info(f'Entity spawned: {name} ({result.status_message})')
            self.entities[name] = result.status_message
        else:
            self.node.get_logger().error(f'Failed to spawn entity: {result.status_message}')

class TestScenario(Scenario):
    def __init__(self, spawn_client: Client, delete_client: Client, node: Node):
        super().__init__(spawn_client, delete_client, node)
        self.manipulator_client = node.create_client(ManipulatorMoveTo, '/manipulator_move_to')
        self.manipulator_busy = False

    def reset(self):
        super().reset()

        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position = Point(x=0.4, y=float(i)/10 + 0.15, z=0.1)
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            pose_transformed = self.node.tf2_buffer.transform(pose, 'odom', timeout=rclpy.time.Duration(seconds=5.0))

            self.spawn_entity('apple', f'apple{i}', pose_transformed.pose)
        
        self.manipulator_busy = False
        self.manipulator_queue = []
    
    def pose_transformed(self, pose: Pose):
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = 'odom'
        pose = self.node.tf2_buffer.transform(pose_stamped, 'world', timeout=rclpy.time.Duration(seconds=5.0)).pose
        pose.orientation = Quaternion(x=0.923880, y=-0.382683, z=0.0, w=0.0)
        return pose

    def move_to_the_left(self, name: str):
        pose = self.get_entity_pose(name)
        pose.position.z += 0.1

        req = ManipulatorMoveTo.Request()
        req.initial_gripper_state = True
        req.target_pose.pose = self.pose_transformed(pose)
        req.final_gripper_state = False
        self.manipulator_queue.append(req)

        pose.position.y -= 0.6
        req = ManipulatorMoveTo.Request()
        req.initial_gripper_state = False
        req.target_pose.pose = self.pose_transformed(pose)
        req.final_gripper_state = True
        self.manipulator_queue.append(req)
    
    def move_callback(self, future: Future):
        result = future.result()
        if result.success:
            self.node.get_logger().debug(f'Move performed')
        else:
            self.node.get_logger().error(f'Failed to perform move')
        self.manipulator_busy = False

    def calculate_progress(self):
        return sum(1 for name in self.entities if self.pose_transformed(self.get_entity_pose(name)).position.y < 0.0) / len(self.entities)

    def step(self):
        if len(self.entities) == 0: # The entities are not spawned yet
            return 0.0, False
        
        progress = self.calculate_progress()

        if not self.manipulator_busy:
            if len(self.manipulator_queue) == 0:
                for name in self.entities:
                    pose = self.pose_transformed(self.get_entity_pose(name))
                    if pose.position.y > 0.0:
                        self.move_to_the_left(name)
                        break

            if len(self.manipulator_queue) > 0:
                req = self.manipulator_queue.pop(0)
                self.manipulator_busy = True
                self.manipulator_client.call_async(req).add_done_callback(self.move_callback)
        
        return progress, progress >= 1.0 and not self.manipulator_busy


def main(args=None):
    rclpy.init(args=args)

    manager = ScenarioManager(TestScenario)

    executor = MultiThreadedExecutor(2)
    executor.add_node(manager)
    executor.spin()

    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
