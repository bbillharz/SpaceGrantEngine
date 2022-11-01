from typing import List


from rclpy import Node


class Engine:
    def __init__(self) -> None:
        self._nodes: List[Node] = []
    
    def main(self):
        rclpy.init()

        map(lambda n: rclpy.spin(n), self._nodes)

    def generate_launch():
        pass


def generate_launch_description(engine: Engine):
    return engine.generate_launch()
