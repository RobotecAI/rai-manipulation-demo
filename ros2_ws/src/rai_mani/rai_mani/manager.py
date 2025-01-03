import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from rclpy.task import Future

from geometry_msgs.msg import Point, Quaternion

from tf2_ros import Buffer, TransformListener

from rai_interfaces.srv import ManipulatorMoveTo

from langchain_core.messages import HumanMessage

from rai.agents.conversational_agent import create_conversational_agent
from rai.node import RaiBaseNode
from rai.tools.ros.manipulation import GetObjectPositionsTool, MoveToPointTool
from rai.tools.ros.native import GetCameraImage, Ros2GetTopicsNamesAndTypesTool
from rai.utils.model_initialization import get_llm_model

from threading import Thread

from rai_mani.scenarios.scenario_base import ScenarioBase

class ScenarioManager(Node):
    """
    A class responsible for playing the scenarios
    """
    def __init__(self, scenario_types):
        """
        Initializes the ScenarioManager
        
        Args:
            scenario_types: A list of scenario classes to play
        """
        super().__init__('scenario_manager')
        self.scenario_types = scenario_types

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.tf2_buffer = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer, self)

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.scenario: ScenarioBase = None
        self.current_scenario = 0
        self.agent_thread: Thread = None
        self.manipulator_ready = False
        self.scores = []
    
    def _init_scenario(self):
        self.scenario = self.scenario_types[self.current_scenario](self.spawn_client, self.delete_client, self)
        self.manipulator_ready = False
        request = ManipulatorMoveTo.Request()
        request.target_pose.pose.orientation = Quaternion(x=0.923880, y=-0.382683, z=0.0, w=0.0)
        request.target_pose.pose.position = Point(x=0.2, y=0.0, z=0.2)
        def callback(future: Future):
            self.manipulator_ready = True
            self.scenario.reset()
        self.scenario.manipulator_client.call_async(request).add_done_callback(callback)

    def _terminate_scenario(self):
        self.get_logger().info(f'Scenario terminated with score {self.scores[-1]}')
        self.scenario = None
        if self.current_scenario == len(self.scenario_types) - 1:
            self.get_logger().info(f'All scenarios are completed, with scores: {self.scores}')
            self.executor.shutdown()
        self.current_scenario = (self.current_scenario + 1) % len(self.scenario_types)
        self.manipulator_ready = False

    def timer_callback(self):
        if self.scenario == None:
            self._init_scenario()

        if not self.manipulator_ready:
            return

        progress, terminated = self.scenario.step()
        self.get_logger().info(f'Task progress: {progress}')
        if terminated and not (self.agent_thread and self.agent_thread.is_alive()):
            self.scores.append(progress)
            self._terminate_scenario()
            return
        if self.agent_thread and not self.agent_thread.is_alive():
            self.get_logger().info('Agent failed to fulfill the task, terminating the scenario.')
            self.scores.append(progress)
            self._terminate_scenario()

class RaiBenchmarkManager(ScenarioManager):
    """
    A class responsible for playing the scenarios and running the conversational agent for each scenario
    """
    def __init__(self, scenario_types):
        super().__init__(scenario_types)
        self.agent = None

    def _init_scenario(self):
        super()._init_scenario()
        
        self.rai_node = RaiBaseNode(node_name="manipulation_demo")
        self.rai_node.declare_parameter("conversion_ratio", 1.0)
        self.rai_node.qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

        tools = [
            GetObjectPositionsTool(
                node=self.rai_node,
                target_frame="panda_link0",
                source_frame="RGBDCamera5",
                camera_topic="/color_image5",
                depth_topic="/depth_image5",
                camera_info_topic="/color_camera_info5",
            ),
            MoveToPointTool(node=self.rai_node, manipulator_frame="panda_link0"),
            GetCameraImage(node=self.rai_node),
            Ros2GetTopicsNamesAndTypesTool(node=self.rai_node),
        ]

        llm = get_llm_model(model_type="complex_model")

        system_prompt = """
        You are a robotic arm with interfaces to detect and manipulate objects.
        Here are the coordinates information:
        x - front to back (positive is forward)
        y - left to right (positive is right)
        z - up to down (positive is up)

        Before starting the task, make sure to grab the camera image to understand the environment.
        """

        self.agent = create_conversational_agent(
            llm=llm,
            tools=tools,
            system_prompt=system_prompt,
        )

        def run_agent():
            self.agent.invoke({"messages": [HumanMessage(content=self.scenario.get_prompt())]})["messages"][-1].pretty_print()
        
        self.agent_thread = Thread(target=run_agent)
        self.agent_thread.start()

