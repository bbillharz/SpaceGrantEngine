from functools import partial

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from reader import OakReader
from sensor_msgs.msg import Image


class OakNode(Node):
    def __init__(self):
        Node.__init__(self, "oak")
        self._custom_publishers = {}
        self._bridge = CvBridge()

        def pcl_callback(data):
            print("Just ogt some data this is a placeholder")

        def image_callback(data, name, image_type: str = "bgr8"):
            if name not in self._custom_publishers:
                self._custom_publishers[name] = self.create_publisher(
                    Image, f"oak/{name}", 1
                )
            image = data.getCvFrame()
            bridge_frame = self._bridge.cv2_to_imgmsg(image, image_type)
            self._custom_publishers[name].publish(bridge_frame)

        self.oak_reader = OakReader(
            pcl_callback,
            partial(image_callback, name="disp", image_type="mono8"),
            partial(image_callback, name="left", image_type="mono8"),
            partial(image_callback, name="right", image_type="mono8"),
        )


def main():
    rclpy.init(args=None)
    oak = OakNode()
    rclpy.spin(oak)
    oak.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
