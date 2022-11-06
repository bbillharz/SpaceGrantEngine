from typing import Any, List, Optional
from ..abstract_node import AbstractNode


class ObjectDetectionNode(AbstractNode):
    """Node for handling object detection"""
    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)
        self._sensor_data: SensorData = None
        self._objects: List[Any] = []
        self._object_publisher: Optional[Publisher] = None
        self._sensor_subscriber: Optional[Subscription] = None

    def _main(self):
        self._object_publisher = self.publish("objects", [])
        self._sensor_subscriber = self.subscribe(
            "sensor data", self._sensor_callback, SensorData
        )

    def _sensor_callback():
        pass
