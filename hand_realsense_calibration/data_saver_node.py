import rclpy
from rclpy.node import Node
import logging
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

# msgs
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header

# add
import numpy as np
from cv_bridge import CvBridge
import cv2
import open3d as o3d
from pypcd4 import PointCloud


class DataSaverNode(Node):
    def __init__(self):
        super().__init__('data_saver_node')
        self.init_param()
        self.init_value()

        # qos_policy = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.sub_image = self.create_subscription(Image(), '/input_rgbd', self.CB_main, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.tf_broadcaster = TransformBroadcaster(self)

    def CB_main(self, msg):
        # trans = self.tf_buffer.lookup_transform(self.frame_id_ground, self.frame_id_depth, rclpy.time.Time())

        current_time = self.get_clock().now()
        time_since_last_publish = (current_time - self.last_publish_time).nanoseconds / 1e9  # 秒に変換

        if time_since_last_publish >= self.pub_interval:
            self.process(msg)
            self.last_publish_time = current_time

    def get_pose_marker2cam(self):
        pass

    def init_param(self):
        self.declare_parameter('frame_id_map', "map")
        self.declare_parameter('frame_id_hand', "hand")
        self.declare_parameter('frame_id_cam', "camera_color_optical_frame")
        self.declare_parameter('camera_param.width',640)
        self.declare_parameter('camera_param.height',260)
        self.declare_parameter('camera_param.k', 
                               [618.33599854,   0.0,         311.00698853,   
                                0.0,         618.52191162,   239.00808716,
                                0.0,           0.0,           1.0        ])
        self.declare_parameter('camera_param.d', [0.0, 0.0, 0.0 , 0.0, 0.0])
        self.frame_id_depth = self.get_parameter('frame_id_map').get_parameter_value().string_value
        self.frame_id_depth = self.get_parameter('frame_id_hand').get_parameter_value().string_value
        self.frame_id_depth = self.get_parameter('frame_id_cam').get_parameter_value().string_value
        self.cam_w = self.get_parameter('camera.width').get_parameter_value().integer_value
        self.cam_h = self.get_parameter('camera.height').get_parameter_value().integer_value
        self.cam_k = self.get_parameter('camera.k').get_parameter_value().double_array_value
        self.cam_d = self.get_parameter('camera.d').get_parameter_value().double_array_value

    def init_value(self):
        pass

def main():
    rclpy.init()
    node = DataSaverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""



"""