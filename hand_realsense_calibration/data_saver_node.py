import rclpy
from rclpy.node import Node
import logging
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

# msgs
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

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
        self.sub_img = self.create_subscription(Image(), '/input_img', self.CB_input_img, 10)
        self.pub_img = self.create_publisher(Image, '/output_image', 10)

        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

    def CB_input_img(self, msg):
        # 
        
        self.msg_input_img = msg
        self.get_pose_cam2board()

        # current_time = self.get_clock().now()
        # time_since_last_publish = (current_time - self.last_publish_time).nanoseconds / 1e9  # 秒に変換

        # if time_since_last_publish >= self.pub_interval:
        #     self.process(msg)
        #     self.last_publish_time = current_time

    def get_pose_cam2board(self):
        img = cv2.cvtColor(CvBridge().imgmsg_to_cv2(self.msg_input_img), cv2.COLOR_BGR2RGB)
        img_undistorted = cv2.undistort(img, self.cam_k, self.cam_d) 
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            img_undistorted, self.aruco_dictionary, parameters=self.aruco_params)
        
        if marker_ids is not None:
            if  len (marker_ids) > 0 : 
                # 補間するCharUco コーナー
                charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    marker_corners, marker_ids, img_undistorted, self.board) 

                # 十分なコーナーが見つかったら、ポーズを推定します
                if charuco_retval: 
                    retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                        charuco_corners, charuco_ids, self.board, self.cam_k, self.cam_d, None , None ) 
                    
                    # ポーズ推定が成功したら、軸を描画します
                    if retval: 
                        cv2.drawFrameAxes(img_undistorted, self.cam_k, self.cam_d, rvec, tvec, length= 0.1 , thickness=3)
                        msg_output_img = CvBridge().cv2_to_imgmsg(cv2.cvtColor(img_undistorted, cv2.COLOR_BGR2RGB),'rgb8')
                        self.pub_img.publish(msg_output_img)


        # return undistorted_image


    # def send_tf_cam2board(self, rvec, tvec):
        # trans = self.tf_buffer.lookup_transform(self.frame_id_ground, self.frame_id_depth, rclpy.time.Time())
    #     t = TransformStamped()
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = self.frame_id_cam
    #     t.child_frame_id = self.frame_id_board

    #     t.transform.translation.x = tvec[0][0]
    #     t.transform.translation.y = tvec[0][1]
    #     t.transform.translation.z = tvec[0][2]
        
    #     t.transform.rotation.x = 0.0
    #     t.transform.rotation.y = 0.0
    #     t.transform.rotation.z = qw
    #     t.transform.rotation.w = qz
    #     self.tf_broadcaster.sendTransform(t)


    def init_param(self):
        self.declare_parameter('frame_id_map', "map")
        self.declare_parameter('frame_id_hand', "hand")
        self.declare_parameter('frame_id_cam', "camera_color_optical_frame")
        self.declare_parameter('frame_id_board', "board")
        self.declare_parameter('camera.width',640)
        self.declare_parameter('camera.height',260)
        self.declare_parameter('camera.k', 
                               [618.33599854,   0.0,         311.00698853,   
                                0.0,         618.52191162,   239.00808716,
                                0.0,           0.0,           1.0        ])
        self.declare_parameter('camera.d', [0.0, 0.0, 0.0 , 0.0, 0.0])
        self.declare_parameter('board.squares_x',7)
        self.declare_parameter('board.squares_y',5)
        self.declare_parameter('board.square_length',0.03)
        self.declare_parameter('board.marker_length',0.015)

        self.frame_id_map = self.get_parameter('frame_id_map').get_parameter_value().string_value
        self.frame_id_hand = self.get_parameter('frame_id_hand').get_parameter_value().string_value
        self.frame_id_cam = self.get_parameter('frame_id_cam').get_parameter_value().string_value
        self.frame_id_board = self.get_parameter('frame_id_board').get_parameter_value().string_value
        self.cam_w = self.get_parameter('camera.width').get_parameter_value().integer_value
        self.cam_h = self.get_parameter('camera.height').get_parameter_value().integer_value
        self.cam_k = self.get_parameter('camera.k').get_parameter_value().double_array_value
        self.cam_d = self.get_parameter('camera.d').get_parameter_value().double_array_value

        self.board_squares_x = self.get_parameter('board.squares_x').get_parameter_value().integer_value
        self.board_squares_y = self.get_parameter('board.squares_y').get_parameter_value().integer_value
        self.board_square_lengh = self.get_parameter('board.square_length').get_parameter_value().double_value
        self.board_marker_lengh = self.get_parameter('board.marker_length').get_parameter_value().double_value

    def init_value(self):
        self.msg_input_img = None
        
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters() 
        self.board = cv2.aruco.CharucoBoard((self.board_squares_x, self.board_squares_y), 
                                       self.board_square_lengh, self.board_marker_lengh, self.aruco_dictionary)
        self.cam_k = np.array(self.cam_k).reshape((3,3))
        self.cam_d = np.array(self.cam_d)

def main():
    rclpy.init()
    node = DataSaverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""



"""