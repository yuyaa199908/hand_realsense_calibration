import rclpy
from rclpy.node import Node
import logging
# msgs
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header

# add
import numpy as np
from cv_bridge import CvBridge
import cv2
import open3d as o3d
# from ctypes import * # convert float to uint32
from pypcd4 import PointCloud
from builtin_interfaces.msg import Time
import struct

class MarkerViewer(Node):
    def __init__(self):
        super().__init__('marker_viewer')
        self.init_param()

        # qos_policy = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.sub_image = self.create_subscription(
                            Image(), 
                            '/input_rgbd', 
                            self.CB_main,
                            10)

    def CB_main(self, msg):
        current_time = self.get_clock().now()
        time_since_last_publish = (current_time - self.last_publish_time).nanoseconds / 1e9  # 秒に変換

        if time_since_last_publish >= self.pub_interval:
            self.process(msg)
            self.last_publish_time = current_time

    def init_param(self):
        self.declare_parameter('frame_id_cam', "camera_color_optical_frame")
        self.declare_parameter('camera.width',640)
        self.declare_parameter('camera.height',260)
        self.declare_parameter('camera.K', 
                               [618.33599854,   0.0,         311.00698853,   
                                0.0,         618.52191162,   239.00808716,
                                0.0,           0.0,           1.0        ])

        self.frame_id_depth = self.get_parameter('frame_id_cam').get_parameter_value().string_value
        self.cam_w = self.get_parameter('camera.width').get_parameter_value().integer_value
        self.cam_h = self.get_parameter('camera.height').get_parameter_value().integer_value
        self.cam_K = self.get_parameter('camera.K').get_parameter_value().double_array_value

def main():
    rclpy.init()
    node = RGBD2CLOUD()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()