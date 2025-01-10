import rclpy

from rai_mani.manager import ScenarioManager

from rai_mani.scenarios.move_to_the_left import MoveToTheLeftAuto
from rai_mani.scenarios.place_on_top import PlaceOnTopAuto
from rai_mani.scenarios.replace_types import ReplaceTypesAuto
from rai_mani.scenarios.longest_object import LongestObjectAuto

from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)

    manager = ScenarioManager([PlaceOnTopAuto, LongestObjectAuto, MoveToTheLeftAuto, ReplaceTypesAuto], list(range(4)))

    executor = MultiThreadedExecutor(2)
    executor.add_node(manager)
    executor.spin()

    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()