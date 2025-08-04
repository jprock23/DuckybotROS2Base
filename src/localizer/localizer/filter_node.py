import rclpy
from rclpy.node import Node
from filterpy.kalman import KalmanFilter, ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
from math import sqrt
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from interfaces.msg import WheelsCmdStamped, WheelEncoderStamped

def HJacobian_at(x):
    """ compute Jacobian of H matrix at x """

    horiz_dist = x[0]
    altitude   = x[2]
    denom = sqrt(horiz_dist**2 + altitude**2)
    return np.array([[horiz_dist/denom, 0., altitude/denom]])

def hx(x):
    """ compute measurement for slant range that
    would correspond to state x.
    """
    
    return (x[0]**2 + x[2]**2) ** 0.5


class Filter_Node(Node):

    def __init__(self, node_name='filter_node'):
        super().__init__(node_name)
        
        self.ekf = ExtendedKalmanFilter(dim_x=3, dim_z=1)
        
        self.ekf.x = np.array([[0.0],
                               [0.0],
                               [0.0]])
        
        self.ekf.F = np.array([[0.0, 1.0, 0.0], 
                               [0.0, 1.0, 0.0],
                               [0.0, 0.0, 0.0]])
        
        self.ekf.R =  np.diag([5**2])
        self.ekf.Q = Q_discrete_white_noise(dim=3, dt=0.1, var=0.1)
        self.ekf.P *= 50
        
        #subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/robot_pose', self.callback, 10)
        
        #publishers

        # listeners
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
    def callback(self, msg: PoseStamped):
        theta = Rotation.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]).as_euler('xyz')[2]
        
        self.ekf.predict()
        self.ekf.update(np.array([[msg.pose.position.x], [msg.pose.position.y], [theta]]), HJacobian=HJacobian_at, Hx=hx)
        
        print(self.ekf.x)

def main():
    rclpy.init()

    filter_node = Filter_Node()

    rclpy.spin(filter_node)

    filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()