from abc import ABC, abstractmethod
from collections import defaultdict
from functools import cached_property
from typing import List, Any, Callable
import asyncio

try:
    import uvloop

    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())
except ModuleNotFoundError:
    pass

from rclpy.node import Node, Publisher, Subscriber
from std_msgs.msg import String


class AbstractNode(ABC, Node):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._publishers: Dict[str, Publisher] = defaultdict(lambda: None) 
        self._subscribers: Dict[str, List[Subscriber]] = defaultdict(lambda: [])

    @cached_property
    def logger(self) -> Any:  # TODO: update hint to be rclpy.logging.Logger?
        """
        Returns the ROS logger instances
        """
        return self.get_logger()

    def _create_publisher(self, topic: str, data: Any, queue_size = 10) -> None:
        assert type(data) != None
        self._publishers[topic] = self.create_publisher(type(data), topic, queue_size)

    def publish(self, topic: str, data: Any) -> None:
        """
        Publishes a given data packet to a given topic
        """
        if self._publishers[topic] == None:
            self._create_publisher(topic, data)
        self._publishers[topic].publish(data)

    def _create_subscriber(self, topic: str, msg_datatype: Any, callback: Callable, queue_size = 10) -> None:
        assert type(data) != None
        self._subscribers[topic].append(self.create_subscriber(type(msg_datatype), topic, callback, queue_size))

    def subscribe(self, topic: str, callback: Callable, msg_datatype: Any = String, queue_size = 10) -> None:
        """
        Adds a callback function as a subscriber to a given topic
        """
        self._create_subscriber(topic, msg_datatype, callback, queue_size)
