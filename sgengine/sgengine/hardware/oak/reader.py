import time
from collections import deque

import cv2
import numpy as np
import depthai as dai
import depthai_sdk as sdk
from oakutils.calibration import get_camera_calibration
from oakutils.nodes import create_stereo_depth, create_xout, get_nn_point_cloud_buffer
from oakutils.nodes.models import create_point_cloud
from oakutils.tools.parsing import get_mono_sensor_resolution_from_tuple
from oakutils.point_clouds import filter_point_cloud, get_point_cloud_from_np_buffer


RGB_SIZE = (1920, 1080)
RGB_PREVIEW_SIZE = (640, 480)
MONO_SIZE = (640, 400)
VOXEL_SIZE = 0.02


def main():
    calib = get_camera_calibration(RGB_SIZE, MONO_SIZE, True)

    pipeline = dai.Pipeline()

    stereo, left, right = create_stereo_depth(
        pipeline,
        resolution=get_mono_sensor_resolution_from_tuple(MONO_SIZE),
        fps=120,
        lr_check=True,
        extended_disparity=True,
        subpixel=False,
        enable_spatial_filter=True,
    )

    pcl, xin_xyz, start_pcl = create_point_cloud(
        pipeline,
        stereo.depth,
        calib,
        shaves=6,
    )

    pcl_xout = create_xout(pipeline, pcl.out, "pcl")

    with dai.Device(pipeline) as device:
        start_pcl(device)

        pcl_q = device.getOutputQueue("pcl", 2, False)
        fps_buffer = deque(maxlen=10)
        c_buffer = deque(maxlen=10)
        filter_buffer = deque(maxlen=10)
        t0 = time.perf_counter()
        while True:
            pcl_data = pcl_q.get()
            pcl_np = get_nn_point_cloud_buffer(pcl_data)

            c0 = time.perf_counter()
            pcl = get_point_cloud_from_np_buffer(pcl_np)
            c1 = time.perf_counter()
            pcl = filter_point_cloud(pcl, VOXEL_SIZE, downsample_first=True)
            c2 = time.perf_counter()
            c_buffer.append(c1 - c0)
            filter_buffer.append(c2 - c1)
            print(f"Conversion took {np.mean(c_buffer):.3f}s")
            print(f"Filtering took {np.mean(filter_buffer):.3f}s")

            t1 = time.perf_counter()
            fps_buffer.append(t1 - t0)
            t0 = t1
            print(f"FPS: {1 / np.mean(fps_buffer):.2f}")


if __name__ == "__main__":
    print("REQUIRES OAKUTILS 1.2.1+ FROM THE DEV BRANCH")
    main()
