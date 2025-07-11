import math
import sys

from cv2 import transform

from ament_index_python import get_package_share_directory

sys.path.append(get_package_share_directory("camera") + "/include")

from geometry_related import euler_to_quat_ros, quat_ros_multiply
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


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


class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):

        super().__init__('static_camera_to_map_broadcaster')
        transformation = Transform()

        transformation.translation.x = 0.6173489079780523
        transformation.translation.y = -0.623187092269024
        transformation.translation.z = 1.9579913

        quat = euler_to_quat_ros(math.pi, 0, 0)

        q = quat_ros_multiply(quat, euler_to_quat_ros(0, 0, 0))

        transformation.rotation = q

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms(transformation)


    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'ceil_camera'

        t.transform = transformation

        self.tf_static_broadcaster.sendTransform(t)


def main():
    rclpy.init()

    broadcaster_node = StaticFramePublisher()

    rclpy.spin(broadcaster_node)

    rclpy.shutdown()

    if __name__ == '__main__':
        main()