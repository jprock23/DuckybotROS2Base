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


#ros2 topic pub --once /robot_pose geometry_msgs/msg/PoseStamped "{pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0}}}"
class Filter_Node(Node):

    def __init__(self, node_name='filter_node'):
        super().__init__(node_name)
        
        #x = (x, y, theta) u = (v, omega)
        dt = 0.1
        self.ekf = ExtendedKalmanFilter(dim_x=3, dim_z=1)
        
        self.ekf.x = np.array([[0.0],
                               [0.0],
                               [0.0]]) #state vector (x, y, theta)
        
        self.ekf.F = np.array([[1.0, 0.0, 0.0], 
                               [0.0, 1.0, 0.0],
                               [0.0, 0.0, 1.0]]) #transition function
        
        
        self.ekf.R =  np.diag([5**2]) # measurment covariance
        self.ekf.Q = Q_discrete_white_noise(dim=3, dt=dt, var=0.1) # noise?
        self.ekf.P = np.array([[0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0]
                               ])# covariances of state vector
        

        self.filter = KalmanFilter(dim_x=3, dim_z=3)
        
        self.filter.x = np.array([[0.0],
                               [0.0],
                               [0.0]]) #state vector (x, y, theta)
        
        self.filter.F = np.array([[1.0, 0.0, 0.0], 
                               [0.0, 1.0, 0.0],
                               [0.0, 0.0, 1.0]]) #transition function
        
        
        self.filter.R =  np.diag([5**2, 5**2, 1**2]) # measurment covariance
        self.filter.Q = Q_discrete_white_noise(dim=3, dt=0.1, var=0.1) # noise?
        self.filter.P = np.array([[0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0]])
        #subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/robot_pose', self.callback, 10)
        
        #publishers
        self.estimate_pub = self.create_publisher(PoseStamped, '/estimate_pose', 10)

        # listeners
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
    def callback(self, msg: PoseStamped):
        theta = Rotation.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]).as_euler('xyz')[2]
        
        # self.ekf.predict()
        # self.ekf.update(np.array([[msg.pose.position.x], [msg.pose.position.y], [theta]]), HJacobian=HJacobian_at, Hx=hx)
        self.filter.predict()
        self.filter.update(np.array([[msg.pose.position.x], [msg.pose.position.y], [theta]]))
        
        print(self.filter.x)
        msg = PoseStamped()

        msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id = 'chassis'

        msg.pose.position.x = float(self.filter.x[0])
        msg.pose.position.y = float(self.filter.x[1])
        msg.pose.position.z = 0.

        quat_list = Rotation.from_euler('xyz', [0., 0., float(self.filter.x[2])]).as_quat()

        msg.pose.orientation.x = quat_list[0]
        msg.pose.orientation.y = quat_list[1]
        msg.pose.orientation.z = quat_list[2]
        msg.pose.orientation.w = quat_list[3]

        self.estimate_pub.publish(msg)



def main():
    rclpy.init()

    filter_node = Filter_Node()

    rclpy.spin(filter_node)

    filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()