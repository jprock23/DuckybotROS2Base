import cv2
from cv_bridge import CvBridge
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
import math
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Quaternion, Vector3
from interfaces.msg import TagPoseStamped

from tf2_ros import TransformBroadcaster


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class Tag_Detector_Node(Node):
    def __init__(self):
        super().__init__("tag_detector_node")

        description = ParameterDescriptor(description="Whether the input image from the camera should be flipped or not before tag detection")
        self.declare_parameter('flip', True, descriptor=description)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.bridge = CvBridge()

        calibration_file = '/home/jproque/dbbase_ws/src/camera/config/calibration/ost.yaml'

        with open(calibration_file, "r") as yamlFile:
            data = yaml.safe_load(yamlFile)
        self.mtx = np.reshape(data["camera_matrix"]["data"], (3, 3))
        self.dst = np.array(data["distortion_coefficients"]["data"])
        self.marker_size = 0.096
        self.img = None

        #Publishers
        self.publisher = self.create_publisher(TagPoseStamped, "/camera/tag_pose", 10)
        self.rejpublisher = self.create_publisher(Image, "/camera/rejected", 10)

        self.timer = self.create_timer(0.5, self.pose_pub)

        #Subscriptions
        self.subscriber = self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.img_sub, 10)

        #Broadcasters
        self.tag_broadcaster = TransformBroadcaster(self)


    def img_sub(self,msg):
        if self.get_parameter('flip').get_parameter_value().bool_value:
            data = cv2.flip(self.bridge.compressed_imgmsg_to_cv2(msg), 1)
        else:
            data = self.bridge.compressed_imgmsg_to_cv2(msg)

        self.img = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
        #Sharpen kernel
        kernel = np.array( [[0, -1, 0],
                            [-1, 5,-1],
                            [0, -1, 0]])

        sharpened = cv2.filter2D(self.img, -1, kernel)
        self.img = sharpened


    def getTagPoses(self):

        params = cv2.aruco.DetectorParameters()

        poses = []

        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, params)
        (corners, ids, rejected) = detector.detectMarkers(self.img)

        cv2.aruco.drawDetectedMarkers(self.img, corners, ids)

        print("Detected Markers:: ", ids)

        if(len(corners) > 0):
            for(markedCorner, markedID) in zip(corners, ids):
                result = cv2.aruco.estimatePoseSingleMarkers(markedCorner, self.marker_size, self.mtx, self.dst)  
                rvec = result[0]
                tvec = result[1]
                
                cv2.drawFrameAxes(self.img, self.mtx, self.dst, rvec, tvec, 0.05)
                poses.append([tvec, rvec, markedID])

        msg = self.bridge.cv2_to_imgmsg(self.img)

        self.rejpublisher.publish(msg)
        return poses


    def pose_pub(self):
        msg = TagPoseStamped()
        if self.img.size != 0:
            tags = self.getTagPoses()
            
            for i in range(len(tags)):
                msg.header = Header()
                msg.header.frame_id = 'ceil_camera'
                msg.header.stamp = self.get_clock().now().to_msg()

                msg.tag_id = tags[i][2][0].item()

                pose = Pose()
                pose.position.x = tags[i][0][0][0][0]
                pose.position.y = tags[i][0][0][0][1]
                pose.position.z = tags[i][0][0][0][2]

                quat = quaternion_from_euler(tags[i][1][0][0][0], tags[i][1][0][0][1], tags[i][1][0][0][2])

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                msg.pose = pose

                self.tag_broadcaster.sendTransform(self.create_transform_stamped(tags[i]))
                self.publisher.publish(msg)


    def create_transform_stamped(self, tag):
        t = TransformStamped()

        t.header = Header()
        t.header.frame_id = 'ceil_camera'
        t.header.stamp = self.get_clock().now().to_msg()
                
        t.child_frame_id = f'tag_{tag[2][0].item()}'

        pose = Vector3()
        pose.x = tag[0][0][0][0]
        pose.y = tag[0][0][0][1]
        pose.z = tag[0][0][0][2]
       
        quat = quaternion_from_euler(tag[1][0][0][0], tag[1][0][0][1], tag[1][0][0][2])
                                     
        quaternion = Quaternion()
        quaternion.x = quat[0]
        quaternion.y = quat[1]
        quaternion.z = quat[2]
        quaternion.w = quat[3]

        t.transform.translation = pose
        t.transform.rotation = quaternion
        
        return t


def main(args=None):
    rclpy.init(args=args)

    tag_detector_node = Tag_Detector_Node()

    rclpy.spin(tag_detector_node)

    tag_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()