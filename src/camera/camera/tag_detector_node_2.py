import math
import cv2
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    Point,
    Pose, 
    PoseArray, 
    Quaternion,
    Transform,
    TransformStamped,
    Vector3
)
from math import modf, pi, acos
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CompressedImage
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
        self._marker_size = 0.1397
        with open(calibration_file_path, "r") as yaml_file:
            data = yaml.safe_load(yaml_file)
        self._mtx = np.reshape(data["camera_matrix"]["data"], (3, 3))
        self._dst = np.array(data["distortion_coefficients"]["data"])
        self._img = np.zeros(shape=(data["image_width"], data["image_height"], 3), dtype=np.uint8)
        
        # cv2 detector
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 
            params
        ) 
        self.board = cv2.aruco.GridBoard([2, 1], markerLength=self._marker_size, markerSeparation=0.078, dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

        self._bridge = CvBridge()
        
        # subscriber
        self._subscriber = self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.img_processing, 10)

        # broadcaster
        self._camera_to_tag_tf_broadcaster = StaticTransformBroadcaster(self)
        self._has_transform = False

        # publishers
        self._tag_poses_pub = self.create_publisher(PoseArray, "/tag_poses", 10)
        self._tag_pose_markers_pub = self.create_publisher(MarkerArray, "/tag_pose_markers", 10)
        self._timer_period = float(0.25)
        self._timer = self.create_timer(self._timer_period, self.calc_tag_pose)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def make_transform(self, tvecs):
        min_x, min_y = float("inf"), float("inf")
        max_x, max_y = -float("inf"), -float("inf")

        idx_A, idx_C, idx_D = 0, 0, 0
        for i in range(len(tvecs)):
            x,y, z = tvecs[i]
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
        norm_vector = np.cross(np.array(corners[1]) - np.array(corners[2]), np.array(corners[0]) - np.array(corners[2]))
        norm_vector /= np.linalg.norm(norm_vector)
        qx, qy, qz = np.cross(np.array([0.0, 0.0, 1.0]), norm_vector)
        qw = 1.0 + np.dot(np.array([0.0, 0.0, 1.0]), norm_vector)

        tf_msg = TransformStamped()
        tf_msg._header._stamp = self.get_clock().now().to_msg()
        tf_msg._header._frame_id = "ceil_camera"
        tf_msg._child_frame_id = "map"
        tf_msg._transform._translation._x = min_x
        tf_msg._transform._translation._y = max_y
        tf_msg._transform._translation._z = tvecs[idx_A][2]
        tf_msg._transform._rotation._x = qx
        tf_msg._transform._rotation._y = qy
        tf_msg._transform._rotation._z = qz
        tf_msg._transform._rotation._w = qw
        self._camera_to_tag_tf_broadcaster.sendTransform(tf_msg)
        self._has_transform = True

    def img_processing(self, msg: CompressedImage):
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
        (corners, ids, rejected) = self._detector.detectMarkers(self._img)
        # self._detector.refineDetectedMarkers(self._img, self.board, corners, ids, rejected, self._mtx, self._dst)

        if(len(corners) > 0):
            # cv2 visualization
            frame = cv2.aruco.drawDetectedMarkers(self._img, corners, ids)
            current_time_msg = self.get_clock().now().to_msg()

            # tag_poses = PoseArray()
            # tag_poses._header._stamp = current_time_msg
            # tag_poses._header._frame_id = "ceil_camera"
            tag_ids = []
            tvecs = []
            rvecs = []

            # tag_pose_markers = MarkerArray()
            # do stuff with the pose of each tag
            for (markedCorners, markedID) in zip(corners, ids):
                #standard tag pose calculation following the cv2 documentation
                obj_pts = np.array([
                    [-self._marker_size / 2.0, self._marker_size / 2.0, 0.0],
                    [self._marker_size / 2.0, self._marker_size / 2.0, 0.0],
                    [self._marker_size / 2.0, -self._marker_size / 2.0, 0.0],
                    [-self._marker_size / 2.0, -self._marker_size / 2.0, 0.0]
                ])
                retval, rvec, tvec = cv2.solvePnP(obj_pts, np.squeeze(markedCorners), self._mtx, self._dst, cv2.SOLVEPNP_IPPE_SQUARE)
                # obj_pts, img_pts = self.board.matchImagePoints(corners, ids)
                # retval, rvec, tvec = cv2.solvePnP(obj_pts, np.squeeze(markedCorners), self._mtx, self._dst)
            
                if (retval > 0):
                    frame = cv2.drawFrameAxes(frame, self._mtx, self._dst, rvec, tvec, self._marker_size, 2)
                    tvecs.append(np.squeeze(tvec))
                    rvecs.append(np.squeeze(rvec))

            if (not self._has_transform and (len(tvecs) >= 3)):
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
                tag_poses_camera_frame = PoseArray()
                tag_poses_camera_frame._header._stamp = self.get_clock().now().to_msg()
                tag_poses_camera_frame._header._frame_id = "map"
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
                    tag_pose._position = Point(
                        x = transformed_pt[0],
                        y = transformed_pt[1],
                        z = transformed_pt[2],
                    )

                    quat_list = quaternion_multiply([camera_to_map_tf.rotation.x, camera_to_map_tf.rotation.y, camera_to_map_tf.rotation.z,camera_to_map_tf.rotation.w], rmtx_as_quat)
                    rotation_as_euler = Rotation.from_quat(quat_list).as_euler('xyz')
                    quat_list = Rotation.from_euler('xyz', [0, 0, rotation_as_euler[2]]).as_quat()

                    tag_pose._orientation = Quaternion(
                        x = quat_list[0],
                        y = quat_list[1],
                        z = quat_list[2],
                        w = quat_list[3]
                    )

                    # tag_pose._orientation = Quaternion(
                    #     x = rmtx_as_quat[0],
                    #     y = rmtx_as_quat[1],
                    #     z = rmtx_as_quat[2],
                    #     w = rmtx_as_quat[3]
                    # )
                    tag_poses_camera_frame._poses.append(tag_pose)




                self._tag_poses_pub.publish(tag_poses_camera_frame)
                cv2.imshow("test", frame)
                cv2.waitKey(1)
                    
                    # T_(camera->tag)
                    # camera_to_tag_tvec = np.subtract(
                    #     np.array(tag_to_camera_tvec),
                    #     np.array([0.0, 0.0, 0.0])
                    # )
                    # camera_to_tag_quat = quaternion_multiply(
                    #     tag_to_camera_quat,
                    #     ((Rotation.from_quat([0.0, 0.0, 0.0, 1.0])).inv()).as_quat()
                    # )

                    # camera_to_tag_euler = Rotation.from_quat(camera_to_tag_quat).as_euler("xyz")
                    # print("old (xyz)", 0, camera_to_tag_euler)
                    # print("old (XYZ)", Rotation.from_quat(camera_to_tag_quat).as_euler("XYZ"))
                    # camera_to_tag_euler[0] = pi
                    # camera_to_tag_euler[1] = 0.0
                    # print("new", 0, camera_to_tag_euler)
                    # camera_to_tag_quat = Rotation.from_euler("xyz", camera_to_tag_euler, degrees=False).as_quat()

                    # camera_to_tag_tf_msg = TransformStamped()
                    # camera_to_tag_tf_msg._header._stamp = current_time_msg
                    # camera_to_tag_tf_msg._header._frame_id = "ceil_camera"
                    # camera_to_tag_tf_msg._child_frame_id = "tag_{}".format(markedID[0])
                    # camera_to_tag_tf_msg._transform._translation = Vector3(
                    #     x=float(camera_to_tag_tvec[0]),
                    #     y=float(camera_to_tag_tvec[1]),
                    #     z=float(camera_to_tag_tvec[2])
                    # )
                    # camera_to_tag_tf_msg._transform._rotation = Quaternion(
                    #     x=float(camera_to_tag_quat[0]),
                    #     y=float(camera_to_tag_quat[1]),
                    #     z=float(camera_to_tag_quat[2]),
                    #     w=float(camera_to_tag_quat[3])
                    # )
                    # self._camera_to_tag_tf_broadcaster.sendTransform(camera_to_tag_tf_msg)
        # publish stuff
        # tag_poses_msg = TagPoseArray()
        # tag_poses_msg._poses = tag_poses
        # tag_poses_msg._ids = tag_ids
        
        # self._tag_pose_markers_pub.publish(tag_pose_markers)
        # cv2 visualization
        

def main(args=None):
    rclpy.init(args=args)
    tag_detector_node = TagDetectorNode("tag_detector_node", "/home/jproque/dbbase_ws/src/camera/config/calibration/ost.yaml")
    rclpy.spin(tag_detector_node)
    tag_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
