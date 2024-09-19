import rclpy
from rclpy.node import Node
import logging
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

# msgs
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped, PolygonStamped, Point32

# add
import numpy as np
from cv_bridge import CvBridge
import cv2
import open3d as o3d
from pypcd4 import PointCloud
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import Time

class DataSaverNode(Node):
    def __init__(self):
        super().__init__('data_saver_node')
        self.init_param()
        self.init_value()

        # qos_policy = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.sub_img = self.create_subscription(Image(), '/input_img', self.CB_input_img, 10)
        self.pub_img = self.create_publisher(Image, '/output_image', 10)
        self.pub_polygon_stamped = self.create_publisher(PolygonStamped,"/output_polygon_stamped",10)
        
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
            # 補間するCharUco コーナー
            charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, img_undistorted, self.board)

            # 十分なコーナーが見つかったら、ポーズを推定します
            if charuco_retval: 
                # self.get_logger().info(f'{charuco_corners},{charuco_ids}')

                retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, self.board, self.cam_k, self.cam_d, None , None ) 
                
                # ポーズ推定が成功したら、軸を描画します
                if retval: 
                    for i in range(len(marker_ids)):
                        cv2.line(img_undistorted, (int(marker_corners[i][0,0,0]),int(marker_corners[i][0,0,1])), 
                                    (int(marker_corners[i][0,1,0]),int(marker_corners[i][0,1,1])), (0, 255, 0))
                        cv2.line(img_undistorted, (int(marker_corners[i][0,1,0]),int(marker_corners[i][0,1,1])), 
                                    (int(marker_corners[i][0,2,0]),int(marker_corners[i][0,2,1])), (0, 255, 0))
                        cv2.line(img_undistorted, (int(marker_corners[i][0,2,0]),int(marker_corners[i][0,2,1])), 
                                    (int(marker_corners[i][0,3,0]),int(marker_corners[i][0,3,1])), (0, 255, 0))
                        cv2.line(img_undistorted, (int(marker_corners[i][0,3,0]),int(marker_corners[i][0,3,1])), 
                                    (int(marker_corners[i][0,0,0]),int(marker_corners[i][0,0,1])), (0, 255, 0))
                        cv2.putText(img_undistorted, f'id={marker_ids[i][0]}', (int(marker_corners[i][0,0,0]),int(marker_corners[i][0,0,1])), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), thickness=3)
                    for i in range(len(charuco_ids)):
                        cv2.drawMarker(img_undistorted, (int(charuco_corners[i,0,0]), int(charuco_corners[i,0,1])), 
                                       (0,0,255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=1, line_type=cv2.LINE_8)
                    cv2.drawFrameAxes(img_undistorted, self.cam_k, self.cam_d, rvec, tvec, length= 0.1 , thickness=3)
                    
                    msg_output_img = CvBridge().cv2_to_imgmsg(cv2.cvtColor(img_undistorted, cv2.COLOR_BGR2RGB),'rgb8')
                    self.pub_img.publish(msg_output_img)
                    
                    # self.get_logger().info(f'{rvec},{tvec}')
                    # rot = Rotation.from_euler("xyz",rvec.reshape((1,3)))
                    # r_matrix = rot.as_matrix()
                    # edge0 = tvec
                    # edge1 = edge0 + np.dot(r_matrix, [self.board_square_lengh * self.board_squares_x, 0, 0])
                    # edge2 = edge1 + np.dot(r_matrix, [0, self.board_square_lengh * self.board_squares_y, 0])
                    # edge3 = edge0 + np.dot(r_matrix, [0, self.board_square_lengh * self.board_squares_y, 0])

                    # msg_output_poly = PolygonStamped()
                    # msg_output_poly.header.frame_id =  self.frame_id_cam
                    # now = self.get_clock().now()
                    # msg_output_poly.header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
                    # for i,edge in enumerate([edge0,edge1,edge2,edge3,edge0]):
                    #     p = Point32()
                    #     p.x, p.y, p.z = edge[0,0], edge[1,0], edge[2,0]
                    #     msg_output_poly.polygon.points.append(p)
                    # self.pub_polygon_stamped.publish(msg_output_poly)
                                            

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
        self.declare_parameter('camera.d', [-0.057686787098646164, 0.06685127317905426, -0.0002468553720973432 , 0.0007276988471858203, -0.021987270563840866])
        self.declare_parameter('board.squares_x',7)
        self.declare_parameter('board.squares_y',5)
        self.declare_parameter('board.square_length',0.04)
        self.declare_parameter('board.marker_length',0.02)

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