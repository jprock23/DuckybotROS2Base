import math
from tracemalloc import start
import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge

from math import modf, pi, acos
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CompressedImage

from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from interfaces.msg import TagPoseArray

def quaternion_multiply(q0: np.ndarray, q1: np.ndarray) -> np.ndarray:
    # from https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html
    # q encoded as [x, y, z, w]
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    return np.array([q0q1_x, q0q1_y, q0q1_z, q0q1_w])

class TagDetectorNode(Node):
    ## tags should be mounted tilted to remove orientational ambiguity from parallel projection
    def __init__(self, node_name: str, calibration_file_path: str = "ost.yaml"):
        super().__init__(node_name)
        # ros params
        description = ParameterDescriptor(description="Whether the input image from the camera should be flipped or not before tag detection")
        self.declare_parameter("flip", True, descriptor=description)

        # calibration and image initialization
        # self._marker_size = 0.1397 #5.5 in
        self._marker_size = 0.1016 # 4 in
        with open(calibration_file_path, "r") as yaml_file:
            data = yaml.safe_load(yaml_file)
        self._mtx = np.reshape(data["camera_matrix"]["data"], (3, 3))
        self._dst = np.array(data["distortion_coefficients"]["data"])
        self._img = np.zeros(shape=(data["image_width"], data["image_height"], 3), dtype=np.uint8)
        
        # cv2 detector
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100), 
            params
        ) 

        self.boards = self.generate_boards(18, .028, 2)

        self._bridge = CvBridge()
        
        # subscriber
        self._subscriber = self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.img_processing, 10)

        # broadcaster
        self._camera_to_map_tf_broadcaster = StaticTransformBroadcaster(self)
        self._has_transform = False

        self._camera_to_tag_tf_broadcaster = TransformBroadcaster(self)

        # publishers
        self._tag_poses_pub = self.create_publisher(TagPoseArray, "/tag_poses", 10)
        self._tag_pose_markers_pub = self.create_publisher(MarkerArray, "/tag_pose_markers", 10)
        self._timer = self.create_timer(0.25, self.calc_tag_pose)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def generate_boards(self, num, seperation, start_id) -> list[cv2.aruco.GridBoard]:
        boards = []
        for i in range(start_id - 1, num, 2):
            print("ids:: ", i, " ", i + 1)
            boards.append(cv2.aruco.GridBoard([1, 2], markerLength=self._marker_size, markerSeparation=seperation, dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100),ids=np.array([i, i + 1])))
        return boards

    def make_transform(self, tvecs):
        self._box_width = 0.155
        self._box_height = 0.23
        #defines the transform from the camera to the map
        min_x, min_y = float("inf"), float("inf")
        max_x, max_y = -float("inf"), -float("inf")

        idx_A, idx_C, idx_D = 0, 0, 0
        for i in range(len(tvecs)):
            x,y = tvecs[i]
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            if (x == min_x):
                idx_A = i
            if(y == min_y):
                idx_C = i
            if(y == max_y):
                idx_D = i
        corners = [
            [min_x, max_y, 0.0], #A, bottom right
            [max_x, min_y, 0.0], #C, top left
            [max_x, max_y, 0.0]  #D, top right
        ]
        corners[0][2] = tvecs[idx_A][2]
        corners[1][2] = tvecs[idx_C][2]
        corners[2][2] = tvecs[idx_D][2]

        # magic method from guy https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
        # to find a quartenion representing the rotation from one 3D-vector to another
        norm_vector = np.cross(np.array(corners[1]) - np.array(corners[2]), np.array(corners[0]) - np.array(corners[2]))
        norm_vector /= np.linalg.norm(norm_vector)
        qx, qy, qz = np.cross(np.array([0.0, 0.0, 1.0]), norm_vector)
        qw = 1.0 + np.dot(np.array([0.0, 0.0, 1.0]), norm_vector)

        tf_msg = TransformStamped()
        tf_msg._header._stamp = self.get_clock().now().to_msg()
        tf_msg._header._frame_id = "ceil_camera"
        tf_msg._child_frame_id = "map"
        # define the map origin to be at the bottom right corner
        tf_msg._transform._translation._x = corners[0][0] - (self._box_height)
        tf_msg._transform._translation._y = corners[0][1] + (self._box_width)
        tf_msg._transform._translation._z = corners[0][2]
        tf_msg._transform._rotation._x = qx
        tf_msg._transform._rotation._y = qy
        tf_msg._transform._rotation._z = qz
        tf_msg._transform._rotation._w = qw
        self._camera_to_map_tf_broadcaster.sendTransform(tf_msg)
        self._has_transform = True

    def img_processing(self, msg: CompressedImage):
        #stores published images and processes them for marker detection and pose estiamtion
        self._img = self._bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        # flip image if necessary
        if (self.get_parameter('flip').get_parameter_value().bool_value):
            self._img = cv2.flip(self._img, 1)
        # sharpen the image
        kernel = np.array( [[0, -1, 0],
                            [-1, 5,-1],
                            [0, -1, 0]])
        self._img = cv2.filter2D(self._img, -1, kernel, borderType=cv2.BORDER_CONSTANT)

    def calc_tag_pose(self):
        tvecs = []
        rvecs = []
        tag_ids = []
        current_time_msg = self.get_clock().now().to_msg()

        tag_poses = TagPoseArray()
        tag_poses.poses._header._stamp = current_time_msg
        tag_poses.poses._header._frame_id = "ceil_camera"
        (corners, ids, rejected) = self._detector.detectMarkers(self._img)
    
        for board in self.boards:
            self._detector.refineDetectedMarkers(self._img, board, corners, ids, rejected, self._mtx, self._dst)
            if(len(corners) > 0):
                # cv2 visualization
                frame = cv2.aruco.drawDetectedMarkers(self._img, corners, ids)

                # do stuff with the pose of each tag
                board_ids = []
                board_corners = []
                for (markerCorners, markerID) in zip(corners, ids):
                    if markerID in board.getIds():
                        board_ids.append(markerID)
                        board_corners.append(markerCorners)
                    if len(board_ids) == 2:
                        break

                rvec = None
                tvec = None
                # print(board_ids)
                # print(board_corners)
                if(len(board_ids) > 0):
                    obj_pts, img_pts = board.matchImagePoints(np.array(board_corners), np.array(board_ids))
                    # print('obj_pts', obj_pts)
                    # print('img_pts', img_pts)
                    if(obj_pts is not None and img_pts is not None):
                        retval, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self._mtx, self._dst)
                        if (retval > 0):
                            frame = cv2.drawFrameAxes(frame, self._mtx, self._dst, rvec, tvec, self._marker_size, 2)
                            tvecs.append(np.squeeze(tvec))
                            rvecs.append(np.squeeze(rvec))
                            print(f'id::{board_ids[0]}, tvec:: {tvec}')
                            tag_ids.append(int(np.squeeze(board_ids[0])))

        if (not self._has_transform and (len(tvecs) >= 3)):
            #if no camera_to_map transform is defined, and at least three tags have been seen, define the transformz
            self.make_transform(tvecs)
        camera_to_map_tf = None
        try:
            camera_to_map_tf: Transform = self._tf_buffer.lookup_transform(
                "map",
                "ceil_camera",
                rclpy.time.Time()).transform
        except:
            pass
        if ((len(tvecs) > 0) and (camera_to_map_tf is not None)):
            camera_to_map_mtx = np.eye(N=4)
            camera_to_map_mtx[:3, :3] = Rotation.from_quat(
                [
                    camera_to_map_tf._rotation._x,
                    camera_to_map_tf._rotation._y,
                    camera_to_map_tf._rotation._z,
                    camera_to_map_tf._rotation._w
                ]
            ).as_matrix()
            
            camera_to_map_mtx[:3, 3] = [
                camera_to_map_tf._translation._x,
                camera_to_map_tf._translation._y,
                camera_to_map_tf._translation._z
            ]
            
            for i in range(len(tvecs)):
                tvec = tvecs[i]
                rvec = rvecs[i]
                rmtx, _ = cv2.Rodrigues(rvec) # 3x3 rotation matrix
                rmtx_as_quat = Rotation.from_matrix(rmtx).as_quat()
                homogenous_pt = np.full(shape=(4, 1), fill_value=1.0) # 4x1 homogenous point
                homogenous_pt[:3, 0] = tvec
                transformed_homogeneous_pt = np.matmul(camera_to_map_mtx, homogenous_pt).flatten()
                transformed_pt = [
                    transformed_homogeneous_pt[0] / float(transformed_homogeneous_pt[3]),
                    transformed_homogeneous_pt[1] / float(transformed_homogeneous_pt[3]),
                    0.0
                ]
                
                tag_pose = Pose()
                tag_pose.position = Point(
                    x = transformed_pt[0],
                    y = transformed_pt[1],
                    z = transformed_pt[2],
                )

                quat_list = quaternion_multiply([camera_to_map_tf.rotation.x, camera_to_map_tf.rotation.y, camera_to_map_tf.rotation.z,camera_to_map_tf.rotation.w], rmtx_as_quat)


                tag_pose.orientation = Quaternion(
                    x = quat_list[0],
                    y = quat_list[1],
                    z = quat_list[2],
                    w = quat_list[3]
                )
                tag_poses.poses.poses.append(tag_pose)

                # T_(map->tag)
                map_to_tag_tvec = np.array([
                        tag_pose.position.x,
                        tag_pose.position.y,
                        tag_pose.position.z,
                ])
                    
                map_to_tag_quat = quaternion_multiply(
                    np.array([tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z, tag_pose.orientation.w]),
                    (Rotation.from_quat([0,0,0,1]).inv()).as_quat()
                )
                
                map_to_tag_tf = TransformStamped()
                map_to_tag_tf._header._stamp = current_time_msg
                map_to_tag_tf._header._frame_id = "map"
                map_to_tag_tf._child_frame_id = "tag_{}".format(tag_ids[i])
                
                map_to_tag_tf.transform.translation = Vector3(
                        x=float(map_to_tag_tvec[0]),
                        y=float(map_to_tag_tvec[1]),
                        z=float(map_to_tag_tvec[2])
                    )
                
                map_to_tag_tf.transform.rotation = Quaternion(
                        x=float(map_to_tag_quat[0]),
                        y=float(map_to_tag_quat[1]),
                        z=float(map_to_tag_quat[2]),
                        w=float(map_to_tag_quat[3])
                )
                
                self._camera_to_tag_tf_broadcaster.sendTransform(map_to_tag_tf)

                tag_poses.ids = tag_ids
                self._tag_poses_pub.publish(tag_poses)
        cv2.imshow("test", frame)
        cv2.waitKey(1)
    

def main(args=None):
    rclpy.init(args=args)
    tag_detector_node = TagDetectorNode("tag_detector_node", "/home/jproque/dbbase_ws/src/camera/config/calibration/ost.yaml")
    rclpy.spin(tag_detector_node)
    tag_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
