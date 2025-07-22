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
    TransformStamped,
    Vector3
)
from math import modf, pi
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
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

        # publishers
        self._tag_poses_pub = self.create_publisher(TagPoseArray, "/tag_poses", 10)
        self._tag_pose_markers_pub = self.create_publisher(MarkerArray, "/tag_pose_markers", 10)
        self._timer_period = float(0.25)
        self._timer = self.create_timer(self._timer_period, self.calc_tag_pose)

        # broadcaster
        self._camera_to_tag_tf_broadcaster = TransformBroadcaster(self)


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
        self._detector.refineDetectedMarkers(self._img, self.board, corners, ids, rejected, self._mtx, self._dst)

        if(len(corners) > 0):
            # cv2 visualization
            frame = cv2.aruco.drawDetectedMarkers(self._img, corners, ids)
            current_time_msg = self.get_clock().now().to_msg()

            tag_poses = PoseArray()
            tag_poses._header._stamp = current_time_msg
            tag_poses._header._frame_id = "ceil_camera"
            tag_ids = []

            tag_pose_markers = MarkerArray()
            # do stuff with the pose of each tag
            for (markedCorners, markedID) in zip(corners, ids):
                #standard tag pose calculation following the cv2 documentation
                obj_pts = np.array([
                    [-self._marker_size / 2.0, self._marker_size / 2.0, 0.0],
                    [self._marker_size / 2.0, self._marker_size / 2.0, 0.0],
                    [self._marker_size / 2.0, -self._marker_size / 2.0, 0.0],
                    [-self._marker_size / 2.0, -self._marker_size / 2.0, 0.0]
                ])
                # retval, rvec, tvec = cv2.solvePnP(obj_pts, np.squeeze(markedCorners), self._mtx, self._dst, cv2.SOLVEPNP_IPPE_SQUARE)
                print("old:: ", obj_pts)
                print("old shape:: ", obj_pts.shape)
                obj_pts, img_pts = self.board.matchImagePoints(corners, ids)
                # print("new:: ", new_obj_pts)
                # print("shape:: ", new_obj_pts.shape)
                # print("img_pts::", img_pts)
                retval, rvec, tvec = cv2.solvePnP(obj_pts, np.squeeze(markedCorners), self._mtx, self._dst)
            
                if (retval > 0):
                    frame = cv2.drawFrameAxes(frame, self._mtx, self._dst, rvec, tvec, self._marker_size, 2)
                    #tag pose in the camera's frame
                    tag_to_camera_tvec = np.squeeze(tvec)
                    tag_to_camera_rvec = np.squeeze(rvec)
                    tag_to_camera_rmtx, _ = cv2.Rodrigues(tag_to_camera_rvec) # 3x3 rotation matrix
                    tag_to_camera_quat = Rotation.from_matrix(tag_to_camera_rmtx).as_quat()
                    # tag_pose = Pose()
                    # tag_pose._position = Point(
                    #     x=float(tag_to_camera_tvec[0]),
                    #     y=float(tag_to_camera_tvec[1]),
                    #     z=float(tag_to_camera_tvec[2])
                    # )
                    # tag_pose._orientation = Quaternion(
                    #     x=float(tag_to_camera_quat[0]),
                    #     y=float(tag_to_camera_quat[1]),
                    #     z=float(tag_to_camera_quat[2]),
                    #     w=float(tag_to_camera_quat[3])
                    # )
                    # tag_poses._poses.append(tag_pose)
                    # tag_ids.append(int(markedID[0]))
                    # visualization marker
                    # tag_pose_marker = Marker()
                    # tag_pose_marker._header._frame_id = "ceil_camera"
                    # tag_pose_marker._header._stamp = current_time_msg
                    # tag_pose_marker._id = int(markedID[0])
                    # tag_pose_marker._type = int(1)
                    # tag_pose_marker._action = int(0)
                    # tag_pose_marker._pose = tag_pose
                    # tag_pose_marker._scale = Vector3(
                    #     x=float(self._marker_size), 
                    #     y=float(self._marker_size), 
                    #     z=0.1*float(self._marker_size)
                    # )
                    # tag_pose_marker._color = ColorRGBA(
                    #     r=float(1.0), 
                    #     g=float(0.0), 
                    #     b=float(0.0), 
                    #     a=float(1.0)
                    # )
                    # tag_pose_marker._lifetime = Duration(
                    #     sec=int(modf(self._timer_period)[1]),
                    #     nanosec=int(1e+9 * modf(self._timer_period)[0])
                    # )
                    # tag_pose_markers._markers.append(tag_pose_marker)
                    # T_(camera->tag)
                    camera_to_tag_tvec = np.subtract(
                        np.array(tag_to_camera_tvec),
                        np.array([0.0, 0.0, 0.0])
                    )
                    camera_to_tag_quat = quaternion_multiply(
                        tag_to_camera_quat,
                        ((Rotation.from_quat([0.0, 0.0, 0.0, 1.0])).inv()).as_quat()
                    )

                    # camera_to_tag_euler = Rotation.from_quat(camera_to_tag_quat).as_euler("xyz")
                    # print("old (xyz)", 0, camera_to_tag_euler)
                    # print("old (XYZ)", Rotation.from_quat(camera_to_tag_quat).as_euler("XYZ"))
                    # camera_to_tag_euler[0] = pi
                    # camera_to_tag_euler[1] = 0.0
                    # print("new", 0, camera_to_tag_euler)
                    # camera_to_tag_quat = Rotation.from_euler("xyz", camera_to_tag_euler, degrees=False).as_quat()

                    camera_to_tag_tf_msg = TransformStamped()
                    camera_to_tag_tf_msg._header._stamp = current_time_msg
                    camera_to_tag_tf_msg._header._frame_id = "ceil_camera"
                    camera_to_tag_tf_msg._child_frame_id = "tag_{}".format(0)
                    camera_to_tag_tf_msg._transform._translation = Vector3(
                        x=float(camera_to_tag_tvec[0]),
                        y=float(camera_to_tag_tvec[1]),
                        z=float(camera_to_tag_tvec[2])
                    )
                    camera_to_tag_tf_msg._transform._rotation = Quaternion(
                        x=float(camera_to_tag_quat[0]),
                        y=float(camera_to_tag_quat[1]),
                        z=float(camera_to_tag_quat[2]),
                        w=float(camera_to_tag_quat[3])
                    )
                    self._camera_to_tag_tf_broadcaster.sendTransform(camera_to_tag_tf_msg)
        # publish stuff
        # tag_poses_msg = TagPoseArray()
        # tag_poses_msg._poses = tag_poses
        # tag_poses_msg._ids = tag_ids
        # self._tag_poses_pub.publish(tag_poses_msg)
        # self._tag_pose_markers_pub.publish(tag_pose_markers)
        # cv2 visualization
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
