from typing import Any, List, Optional
from ..abstract_node import AbstractNode
from abc import abstractmethod


class ObjectDetectionNode(AbstractNode):
    """Node for handling object detection"""
    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)
        self.color_image: Optional[ColorImage] = None
        self.depth_image: Optional[DepthImage] = None
        self.imu_data: Optional[ImuData] = None
        self._objects: List[Any] = []

    def _main(self):
        self.publish("objects", [])
        self.subscribe(
            "color image", self._color_callback, ColorImage
        )
        self.subscribe(
            "depth image", self._depth_callback, DepthImage
        )
        self.subscribe(
            "imu data", self._imu_callback, ImuData
        )

    def _color_callback(self, msg: ColorImage):
        self._sensor_data = msg
        self._find_objects()

    def _depth_callback(self, msg: DepthImage):
        self._sensor_data = msg
        self._find_objects()
    
    def _imu_callback(self, msg: ImuData):
        self._sensor_data = msg
        self._find_objects()

    @abstractmethod
    def _find_objects():
        pass