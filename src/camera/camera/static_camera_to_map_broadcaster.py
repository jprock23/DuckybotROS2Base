import math
import sys

from cv2 import transform

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
        transformation.translation.x = -0.9268597160482871
        transformation.translation.y = -0.8796537877987689
        transformation.translation.z = 1.626891409883178

        transformation.rotation.x = -0.3739951171318041
        transformation.rotation.y = 0.4929225317553039
        transformation.rotation.z = 0.7164978290127474
        transformation.rotation.w = 0.3221581770955524

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms(transformation)

    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'ceil_camera'
        t.child_frame_id = 'map'

        t.transform = transformation

        # t.transform.translation.x = transform.x
        # t.transform.translation.y = float(transformation[3])
        # t.transform.translation.z = float(transformation[4])
        # quat = quaternion_from_euler(
        #     float(transformation[5]), float(transformation[6]), float(transformation[7]))
        # t.transform.rotation.x = quat[0]
        # t.transform.rotation.y = quat[1]
        # t.transform.rotation.z = quat[2]
        # t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)


def main():
    rclpy.init()

    broadcaster_node = StaticFramePublisher()

    rclpy.spin(broadcaster_node)

    rclpy.shutdown()

    if __name__ == '__main__':
        main()