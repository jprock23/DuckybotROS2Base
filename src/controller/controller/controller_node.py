import rclpy
from rclpy.node import Node
from simple_pid import PID
from math import pi

from nav_msgs.msg import Odometry
from interfaces.msg import WheelEncoderStamped, WheelsCmdStamped, Throttle
from geometry_msgs.msg import TwistStamped

class Controller_Node(Node):
    #ros2 topic pub --once /wheels_cmd interfaces/msg/WheelsCmdStamped "{vel_left: 0.0, vel_right: 0.0}"
    def __init__(self):
        super().__init__('controller_node')
        
        kP_linear_l, kI_linear_l, kD_linear_l = 5.25, 0.0, 0.0
        kP_linear_r, kI_linear_r, kD_linear_r = 5.5, 0.0, 0.0

        self.left_linear_controller = PID(kP_linear_l, kI_linear_l, kD_linear_l)
        self.right_linear_controller = PID(kP_linear_r, kI_linear_r, kD_linear_r)
        
        kP_angular, kI_angular, kD_angular = 1.0, 0.0, 0.0
        self.angular_controller = PID(kP_angular, kI_angular, kD_angular)
        
        #Subscribers
        self.cmd_subscription = self.create_subscription(WheelsCmdStamped, '/wheels_cmd', self.set_linear_setpoint, 10)
        # self.angular_cmd_subscription = self.create_subscription(TwistStamped, '/angular_cmd', self.set_angular_setpoint, 10)
        self.velocity_left_subscription = self.create_subscription(TwistStamped, '/left_encoder_node/velocity', self.update_left_vel, 10)
        self.velocity_right_subscription = self.create_subscription(TwistStamped, '/right_encoder_node/velocity', self.update_right_vel, 10)

        #Publishers
        self.executed_cmd_publisher = self.create_publisher(WheelsCmdStamped, '/wheels_cmd_executed', 10)
        self.throttle_publisher = self.create_publisher(Throttle, '/throttles', 10)
        self.twist_publisher = self.create_publisher(TwistStamped, '/robot_twist', 10)

        self.wheel_radius = .0325
        self.track_width = 0.14

        self.prev_Lticks = None
        self.prev_Rticks = None

        self.curr_velL = 0.0
        self.curr_velR = 0.0
        self.angular_vel = 0.0
        self.linear_vel = 0.0
        
        self.max_linear_vel = 0.40
        self.max_angular_vel = 0.25

        self.left_throttle = 0.0
        self.right_throttle = 0.0

        self.timer_period = 1/30.0
        # self.twist_timer = self.create_timer(self.timer_period, self.calculate_twist)
        self.linear_control_timer = self.create_timer(self.timer_period, self.calculate_linear_control)
        # self.angular_control_timer = self.create_timer(self.timer_period, self.calculate_angular_control)


    def update_left_vel(self, msg: TwistStamped):
        self.curr_velL = msg.twist.linear.x


    def update_right_vel(self, msg: TwistStamped):
        self.curr_velR = msg.twist.linear.x


    def calculate_twist(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        
        self.angular_vel = (self.curr_velR - self.curr_velL)/self.track_width
        self.linear_vel = (self.curr_velL + self.curr_velR)/2.0
        
        twist_msg.twist.linear.x = self.linear_vel
        twist_msg.twist.angular.z = self.angular_vel
        
        self.twist_publisher.publish(twist_msg)


    def calculate_angular_control(self):
        control = self.angular_controller(self.angular_vel)
        
        self.left_linear_controller.setpoint = max(-self.max_linear_vel, min(self.linear_vel - control *(self.track_width/2), self.max_linear_vel))
        self.right_linear_controller.setpoint = max(-self.max_linear_vel, min(self.linear_vel + control *(self.track_width/2), self.max_linear_vel))
        

    def calculate_linear_control(self):
        self.left_throttle = self.left_linear_controller(self.curr_velL)
        self.right_throttle = self.right_linear_controller(self.curr_velR)
        # self.left_throttle = 1.0
        # self.right_throttle = 1.0

        print("left_val:: ", self.curr_velL)
        print("right_val:: ", self.curr_velR)

        throttle_msg = Throttle()
        throttle_msg.left_throttle = max(-1.0, min(self.left_throttle, 1.0))
        throttle_msg.right_throttle = max(-1.0, min(self.right_throttle, 1.0))
        self.throttle_publisher.publish(throttle_msg)

        executed_msg = WheelsCmdStamped()
        executed_msg.header.stamp = self.get_clock().now().to_msg()
        executed_msg.header.frame_id = 'base_link'
        executed_msg.vel_left = self.left_throttle
        executed_msg.vel_right = self.right_throttle
        self.executed_cmd_publisher.publish(executed_msg)

        
    def set_linear_setpoint(self, msg: WheelsCmdStamped):
        """callback that subscirbes to the /wheels_cmd topic to get and apply motor controls"""
        self.get_logger().info(f'Time stamp: {msg.header.stamp.sec}, Frame_id: {msg.header.frame_id}, vel_left: {msg.vel_left}, vel_right: {msg.vel_right}')

        self.left_linear_controller.setpoint = max(-self.max_linear_vel, min(msg.vel_left, self.max_linear_vel))
        self.right_linear_controller.setpoint = max(-self.max_linear_vel, min(msg.vel_right, self.max_linear_vel))

        print("left_set::", self.left_linear_controller.setpoint)
        print("right_set::", self.right_linear_controller.setpoint)


    def set_angular_setpoint(self, msg: TwistStamped):
        self.angular_controller.setpoint = max(-self.max_angular_vel, min(msg.twist.angular.z, self.max_angular_vel))
        
        print("angular_set:: ", self.angular_controller.setpoint)

def main():
    rclpy.init()
    
    controller_node = Controller_Node()
    
    rclpy.spin(controller_node)
    
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()