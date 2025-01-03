import rclpy
from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future

from geometry_msgs.msg import Point, Quaternion, PoseStamped

from rai_interfaces.srv import ManipulatorMoveTo

from rai_mani.scenarios.scenario_base import ScenarioBase

class PlaceOnTop(ScenarioBase):
    def __init__(self, spawn_client: Client, delete_client: Client, node: Node):
        self.top_object = None
        self.bot_object = None

        super().__init__(spawn_client, delete_client, node)

    def get_prompt(self):
        return "Place the yellow cube on top of the blue cube. Remember to increase the Z position of the 'drop' task by around 0.2 to avoid collision."

    def reset(self):
        super().reset()

        prefabs = ['apple', 'yellow_cube', 'blue_cube', 'carrot']
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position = Point(x=0.4, y=float(i)/8 - 0.25, z=0.1)
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            pose_transformed = self.node.tf2_buffer.transform(pose, 'odom', timeout=rclpy.time.Duration(seconds=5.0))

            prefab = prefabs[i % len(prefabs)]
            if prefab == 'yellow_cube':
                self.top_object = f'{prefab}{i}'
            elif prefab == 'blue_cube':
                self.bot_object = f'{prefab}{i}'

            self.spawn_entity(prefab, f'{prefab}{i}', pose_transformed.pose)

    def calculate_progress(self):
        if self.top_object == None or self.bot_object == None:
            return 0.0

        top_pose = self.pose_transformed(self.get_entity_pose(self.top_object))
        bot_pose = self.pose_transformed(self.get_entity_pose(self.bot_object))

        goal_position = bot_pose.position
        goal_position.z += 0.05

        def distance(a, b):
            return ((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)**0.5

        return max(0.0, 1.0 - 10.0 * distance(top_pose.position, goal_position))
    
    def step(self):
        if len(self.entities) == 0:
            return 0.0, False
        
        progress = self.calculate_progress()

        return progress, progress >= 0.8 and not self.manipulator_busy

class PlaceOnTopAuto(PlaceOnTop):
    def __init__(self, spawn_client: Client, delete_client: Client, node: Node):
        self.manipulator_busy = False

        super().__init__(spawn_client, delete_client, node)

    def reset(self):
        super().reset()

        self.manipulator_busy = False
        self.manipulator_queue = []
    
    def place_on_top(self, bot_object: str, top_object: str):
        pose = self.get_entity_pose(top_object)
        pose.position.z += 0.1

        req = ManipulatorMoveTo.Request()
        req.initial_gripper_state = True
        req.target_pose.pose = self.pose_transformed(pose)
        req.final_gripper_state = False
        self.manipulator_queue.append(req)

        pose = self.get_entity_pose(bot_object)
        pose.position.z += 0.2
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

    def step(self):
        if len(self.entities) == 0:
            return 0.0, False
        
        progress = self.calculate_progress()
    
        if not self.manipulator_busy:
            if len(self.manipulator_queue) == 0 and progress < 0.8:
                self.place_on_top(self.bot_object, self.top_object)
            
            if len(self.manipulator_queue) > 0:
                req = self.manipulator_queue.pop(0)
                self.manipulator_busy = True
                self.manipulator_client.call_async(req).add_done_callback(self.move_callback)

        return progress, progress >= 0.8 and not self.manipulator_busy