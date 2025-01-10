import rclpy

from rai_mani.manager import RaiBenchmarkManager

from rai_mani.scenarios.move_to_the_left import MoveToTheLeft
from rai_mani.scenarios.place_on_top import PlaceOnTop
from rai_mani.scenarios.replace_types import ReplaceTypes

from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)

    manager = RaiBenchmarkManager([PlaceOnTop, MoveToTheLeft, ReplaceTypes], list(range(3)))

    executor = MultiThreadedExecutor(2)
    executor.add_node(manager)
    executor.spin()

    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
