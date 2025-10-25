import rclpy
from rclpy.node import Node
from simple_pid import PID

from interfaces.msg import WheelsCmdStamped, Throttle
from geometry_msgs.msg import TwistStamped

class Controller_Node(Node):
    #ros2 topic pub --once /wheels_cmd interfaces/msg/WheelsCmdStamped "{vel_left: 0.0, vel_right: 0.0}"
    #ros2 topic pub --once /angular_cmd geometry_msgs/msg/TwistStamped "{twist: {angular: {x: 0.0, y: 0.0, z: 1.5708}}}"

    def __init__(self):
        super().__init__('controller_node')
        
        # --- Parameters ---
        self.declare_parameter('kp', 2.1)
        self.declare_parameter('ki', 0.7)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('output_limits', [-1.0, 1.0])  # throttle range
        control_period = 1/10

        kp = self.get_parameter('kp').get_parameter_value().double_value
        ki = self.get_parameter('ki').get_parameter_value().double_value
        kd = self.get_parameter('kd').get_parameter_value().double_value
        output_limits = self.get_parameter('output_limits').get_parameter_value().double_array_value
        output_limits = tuple(output_limits) if output_limits else (-1.0, 1.0)

        # Controllers
        self.pid_left = PID(kp + 0.05, ki, kd, setpoint=0, sample_time=control_period)
        self.pid_right = PID(kp, ki, kd, setpoint=0, sample_time=control_period)
        self.pid_left.output_limits = output_limits
        self.pid_right.output_limits = output_limits

        # Subscribers
        self.create_subscription(TwistStamped, '/left_encoder_node/velocity', self.update_left_vel, 10)
        self.create_subscription(TwistStamped, '/right_encoder_node/velocity', self.update_right_vel, 10)
        self.create_subscription(WheelsCmdStamped, '/wheels_cmd', self.set_linear_setpoint, 10)
        # self.angular_cmd_subscription = self.create_subscription(TwistStamped, '/angular_cmd', self.set_angular_setpoint, 10)

        # Publishers
        self.throttle_pub = self.create_publisher(Throttle, '/throttles', 10)
        self.executed_cmd_publisher = self.create_publisher(WheelsCmdStamped, '/wheels_cmd_executed', 10)
        self.twist_publisher = self.create_publisher(TwistStamped, '/robot_twist', 10)

        # Internal state
        self.current_left_vel = 0.0
        self.current_right_vel = 0.0
        self.target_left_vel = 0.0
        self.target_right_vel = 0.0
        self.last_left_time = None
        self.last_right_time = None

        self.max_linear_vel = 0.40
        self.max_angular_vel = 0.25

        #timer
        self.timer = self.create_timer(control_period, self.calculate_linear_control)


    def update_left_vel(self, msg: TwistStamped):
        self.current_left_vel = msg.twist.linear.x
        self.last_left_time = msg.header.stamp

    def update_right_vel(self, msg: TwistStamped):
        self.current_right_vel = msg.twist.linear.x
        self.last_right_time = msg.header.stamp
        

    def calculate_linear_control(self):
        left_throttle = self.pid_left(self.current_left_vel)
        right_throttle = self.pid_right(self.current_right_vel)

        if self.pid_left.setpoint == 0.0:
            left_throttle = 0.0
        if self.pid_right.setpoint == 0.0:
            right_throttle = 0.0

        throttle_msg = Throttle()
        throttle_msg.left_throttle = float(left_throttle)
        throttle_msg.right_throttle = float(right_throttle)

        executed_msg = WheelsCmdStamped()
        executed_msg.header.stamp = self.get_clock().now().to_msg()
        executed_msg.header.frame_id = 'base_link'
        executed_msg.vel_left = left_throttle
        executed_msg.vel_right = right_throttle
        self.executed_cmd_publisher.publish(executed_msg)

        self.throttle_pub.publish(throttle_msg)

        
    def set_linear_setpoint(self, msg: WheelsCmdStamped):
        self.target_left_vel = msg.vel_left
        self.target_right_vel = msg.vel_right
        self.pid_left.setpoint = self.target_left_vel
        self.pid_right.setpoint = self.target_right_vel


    def set_angular_setpoint(self, msg: TwistStamped):
        # self.angular_controller.setpoint = max(-self.max_angular_vel, min(msg.twist.angular.z, self.max_angular_vel))
        self.target_omega = msg.twist.angular.z

        print("angular_set:: ", self.target_omega)

def main():
    rclpy.init()
    controller_node = Controller_Node()
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()