from cv2 import sqrt
import wheel
import rclpy
from rclpy.node import Node
from simple_pid import PID

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from interfaces.msg import WheelsCmdStamped

class Navigation_Controller_Node(Node):
    
    def __init__(self):
        super().__init__("navigation_controller_node")
        
        #Subscribers
        self.odom_subscriber = self.create_subscription(PoseWithCovarianceStamped, "/robot_pose", self.update_state, 10)
        self.setpoint_subscriber = self.create_subscription(PoseStamped, "/target", self.set_setpoint, 10)
        
        #Publisher
        self.cmd_publisher = self.create_publisher(WheelsCmdStamped, '/wheels_cmd', 10)

        control_period = 1/30

        #Controllers
        self.pose_controller = PID(Kp=2.0, Ki=0.0, Kd=0.0)
        self.omega_controller = PID(Kp=2.0, Ki=0.0, Kd=0.0)

        self.pose_controller.sample_time = control_period
        self.omega_controller.sample_time = control_period

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.timer = self.create_timer(control_period, self.calculate_control)

        self.min_speed = 0.15
        self.track_width = 0.14

        self.left_val = 0.0
        self.right_val = 0.0

    def update_state(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.x
        self.theta = msg.pose.pose.orientation.z

    def set_setpoint(self, msg: PoseStamped):
        self.pose_controller.setpoint = msg.pose.position.x
        self.omega_controller.setpoint = msg.pose.orientation.z

    def decompose_omega(self, omega):
        vel_left = -omega * self.track_width/2
        vel_right = omega * self.track_width/2

        return (vel_left, vel_right)


    def calculate_control(self):
        omega = self.omega_controller(self.theta)
        vel_left, vel_right = self.decompose_omega(omega)

        wheel_cmd = WheelsCmdStamped()
        wheel_cmd.header.stamp = self.get_clock().now().to_msg()
        wheel_cmd.vel_left = vel_left
        wheel_cmd.vel_right = vel_right

        self.cmd_publisher.publish(wheel_cmd)

        # target_velo = self.pose_controller(self.x)

        # print(target_velo)
        # target_velo = target_velo if abs(target_velo) > self.min_speed else 0.0
        # print(target_velo)

        # wheels_cmd = WheelsCmdStamped()
        # wheels_cmd.header.stamp = self.get_clock().now().to_msg()
        # wheels_cmd.vel_left = target_velo
        # wheels_cmd.vel_right = target_velo

        # self.cmd_publisher.publish(wheels_cmd)
        
def main():
    rclpy.init()
    
    navigation_controller_node=Navigation_Controller_Node()
    
    rclpy.spin(navigation_controller_node)
    
    navigation_controller_node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
