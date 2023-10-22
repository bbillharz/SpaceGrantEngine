from oakutils import ApiCamera
from oakutils.nodes import (create_stereo_depth, create_xout,
                            get_nn_point_cloud_buffer)
from oakutils.nodes.models import create_point_cloud
from oakutils.point_clouds import (filter_point_cloud,
                                   get_point_cloud_from_np_buffer)
from oakutils.tools.parsing import get_mono_sensor_resolution_from_tuple


def pcl_callback(data):
    pcl_buff = get_nn_point_cloud_buffer(data)
    shape = pcl_buff.shape
    pcl = get_point_cloud_from_np_buffer(pcl_buff)
    pcl = filter_point_cloud(pcl, 0.02, downsample_first=True)
    print(f"Got pcl: {shape}")


def image_callback(data):
    data = data.getCvFrame()
    print(f"Got image of size: {data.shape}")


class OakReader:
    def __init__(
        self,
        pcl_callback,
        disparity_callback,
        left_callback,
        right_callback,
        primary_mono_left=True,
        color_size=(1920, 1080),
        mono_size=(640, 400),
        preview_size=(640, 480),
        voxel_size=0.02,
    ):
        self._primary_mono_left = primary_mono_left
        self._color_size = color_size
        self._mono_size = mono_size
        self._preview_size = preview_size
        self._voxel_size = voxel_size
        self._node_allocations = []

        oak = ApiCamera(
            primary_mono_left=self._primary_mono_left,
            color_size=self._color_size,
            mono_size=self._mono_size,
        )
        stereo, left, right = create_stereo_depth(
            oak.pipeline,
            get_mono_sensor_resolution_from_tuple(self._mono_size),
            fps=60,
        )
        pcl, xin_xyz, pcl_device_call = create_point_cloud(
            oak.pipeline,
            stereo.depth,
            oak.calibration,
        )
        oak.add_device_call(pcl_device_call)

        # create callback for point cloud
        xout_pcl = create_xout(oak.pipeline, pcl.out, "pcl")
        oak.add_callback("pcl", pcl_callback)

        # create callback for disparity, left, right image
        xout_disp = create_xout(oak.pipeline, stereo.disparity, "disp")
        oak.add_callback("disp", disparity_callback)
        xout_left = create_xout(oak.pipeline, stereo.rectifiedLeft, "left")
        oak.add_callback("left", left_callback)
        xout_right = create_xout(oak.pipeline, stereo.rectifiedRight, "right")
        oak.add_callback("right", right_callback)

        # add all nodes to allocations
        self._node_allocations.extend([stereo, left, right])
        self._node_allocations.extend([pcl, xin_xyz, xout_pcl])
        self._node_allocations.extend([xout_disp, xout_left, xout_right])

        # start the camera
        oak.start(blocking=False)


def main():
    import time

    OakReader(
        pcl_callback,
        image_callback,
        image_callback,
        image_callback,
    )
    while True:
        time.sleep(0.1)


if __name__ == "__main__":
    from oakutils import __version__ as version

    # ensure version 1.2.1 or higher is used
    if version < "1.2.1":
        raise RuntimeError("Please upgrade oakutils to version 1.2.1 or higher")
    main()
