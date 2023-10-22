from functools import partial

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from reader import OakReader
from sensor_msgs.msg import Image, PointCloud2, PointField

from oakutils.nodes import get_nn_point_cloud_buffer
from oakutils.point_clouds import get_point_cloud_from_np_buffer, filter_point_cloud


# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                  (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                  (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)
 
# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}

def dtype_to_fields(dtype):
    '''Convert a numpy record datatype into a list of PointFields.
    '''
    fields = []
    for field_name in dtype.names:
        np_field_type, field_offset = dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        if np_field_type.subdtype:
            item_dtype, shape = np_field_type.subdtype
            pf.count = np.prod(shape)
            np_field_type = item_dtype
        else:
            pf.count = 1
        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        fields.append(pf)
    return fields

def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy array to a sensor_msgs.msg.PointCloud2.
    '''
    # convert to record array
    dtype = [('x', float), ('y', float), ('z', float)]
    cloud_arr = np.array(cloud_arr, dtype=dtype)

    # make it 2d (even if height will be 1)
    cloud_arr = np.atleast_2d(cloud_arr)
 
    cloud_msg = PointCloud2()
    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    cloud_msg.is_dense = True # assumption since we filtered out nan's
    cloud_msg.data = cloud_arr.tostring()
    return cloud_msg


class OakNode(Node):
    def __init__(self):
        Node.__init__(self, "oak")
        self._custom_publishers = {}
        self._bridge = CvBridge()

        def pcl_callback(data):
            pcl_buff = get_nn_point_cloud_buffer(data)
            open3d_pcl = get_point_cloud_from_np_buffer(pcl_buff)
            open3d_pcl = filter_point_cloud(open3d_pcl, 0.02, downsample_first=True)
            buff_arr = np.asarray(open3d_pcl.points)
            pcl2 = array_to_pointcloud2(buff_arr)
            if "pcl" not in self._custom_publishers:
                self._custom_publishers["pcl"] = self.create_publisher(
                    PointCloud2, "oak/pcl", 1
                )
            self._custom_publishers["pcl"].publish(pcl2)

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
