from typing import Any, List, Optional
from ..abstract_node import AbstractNode
from abc import abstractmethod


class ObjectDetectionNode(AbstractNode):
    """Node for handling object detection"""
    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)
        self._sensor_data: SensorData = None
        self._objects: List[Any] = []
        self._object_publisher: Optional[Publisher] = None
        self._sensor_subscriber: Optional[Subscription] = None

    def _main(self):
        self.publish("objects", [])
        self.subscribe(
            "sensor data", self._sensor_callback, SensorData
        )

    def _sensor_callback(self, msg: SensorData):
        self._sensor_data = msg
        self._find_objects()
        pass

    @abstractmethod
    def _find_objects():
        pass