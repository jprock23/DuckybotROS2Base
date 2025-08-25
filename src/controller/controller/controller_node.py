import rclpy
from rclpy.node import Node
from simple_pid import PID
from math import pi

from nav_msgs.msg import Odometry
from interfaces.msg import WheelEncoderStamped, WheelsCmdStamped, Throttle
from geometry_msgs.msg import TwistStamped

class Controller_Node(Node):
    
    def __init__(self):
        super().__init__('controller_node')
        
        kP_linear, kI_linear, kD_linear = 2.0, 0.0, 0.0
        self.left_linear_controller = PID(kP_linear, kI_linear, kD_linear)
        self.right_linear_controller = PID(kP_linear, kI_linear, kD_linear)
        
        kP_angular, kI_angular, kD_angular = 1.0, 0.0, 0.0
        self.angular_controller = PID(kP_angular, kI_angular, kD_angular)
        
        #Subscribers
        self.cmd_subscription = self.create_subscription(WheelsCmdStamped, '/wheels_cmd', self.set_setpoint, 10)
        self.velocity_left_subscription = self.create_subscription(TwistStamped, '/left_encoder_node/velocity', self.update_left_vel, 10)
        self.velocity_right_subscription = self.create_subscription(TwistStamped, '/right_encoder_node/velocity', self.update_right_vel, 10)

        #Publishers
        self.executed_cmd_publisher = self.create_publisher(WheelsCmdStamped, '/wheels_cmd_executed', 10)
        self.throttle_publisher = self.create_publisher(Throttle, '/throttles', 10)
        self.twist_publisher = self.create_publisher(TwistStamped, '/robot_twist', 10)

        self.wheel_radius = .0325
        self.wheel_base = 0.14

        self.prev_Lticks = None
        self.prev_Rticks = None

        self.curr_velL = 0.0
        self.curr_velR = 0.0

        self.left_throttle = 0.0
        self.right_throttle = 0.0

        self.time_period = 1/30.0
        self.control_timer = self.create_timer(self.time_period, self.calculate_control)
        self.twist_timer = self.create_timer(self.time_period, self.calculate_twist)


    def update_left_vel(self, msg: TwistStamped):
        self.curr_velL = msg.twist.linear.x


    def update_right_vel(self, msg: TwistStamped):
        self.curr_velR = msg.twist.linear.x


    def calculate_twist(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        
        twist_msg.twist.linear.x = (self.curr_velL + self.curr_velR)/2.0
        twist_msg.twist.angular.z = (self.curr_velR - self.curr_velL)/self.wheel_base
        
        self.twist_publisher.publish(twist_msg)


    def calculate_control(self):
        self.left_throttle = self.left_linear_controller(self.curr_velL)
        self.right_throttle = self.right_linear_controller(self.curr_velR)

        print("left_val:: ", self.curr_velL)
        print("right_val:: ", self.curr_velR)

        throttle_msg = Throttle()
        throttle_msg.left_throttle = self.left_throttle
        throttle_msg.right_throttle = self.right_throttle
        self.throttle_publisher.publish(throttle_msg)

        executed_msg = WheelsCmdStamped()
        executed_msg.header.stamp = self.get_clock().now().to_msg()
        executed_msg.header.frame_id = 'base_link'
        executed_msg.vel_left = self.left_throttle
        executed_msg.vel_right = self.right_throttle
        self.executed_cmd_publisher.publish(executed_msg)

        
    def set_setpoint(self, msg: WheelsCmdStamped):
        """callback that subscirbes to the /wheels_cmd topic to get and apply motor controls"""
        self.get_logger().info(f'Time stamp: {msg.header.stamp.sec}, Frame_id: {msg.header.frame_id}, vel_left: {msg.vel_left}, vel_right: {msg.vel_right}')

        self.left_linear_controller.setpoint = max(-0.25, min(msg.vel_left, 0.25))
        self.right_linear_controller.setpoint = max(-0.25, min(msg.vel_right, 0.25))

        print("left_set::", self.left_linear_controller.setpoint)
        print("right_set::", self.right_linear_controller.setpoint)

def main():
    rclpy.init()
    
    controller_node = Controller_Node()
    
    rclpy.spin(controller_node)
    
    rclpy.shutdown()