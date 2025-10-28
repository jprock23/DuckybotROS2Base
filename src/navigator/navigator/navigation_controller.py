import math
import rclpy
from rclpy.node import Node
from simple_pid import PID

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from interfaces.msg import WheelsCmdStamped

class Navigation_Controller_Node(Node):
    #ros2 topipub --once /target geometry_msgs/msg/PoseStamped "{pose: {position: {x: -0.15, y: 0.0, z: 0.0}}}"
    #ros2 topic pub --once /target geometry_msgs/msg/PoseStamped "{pose: {orientation: {x: -0.15, y: 0.0, z: 0.033}}}"
    def __init__(self):
        super().__init__("navigation_controller_node")
        
        #Subscribers
        self.odom_subscriber = self.create_subscription(PoseWithCovarianceStamped, "/robot_pose", self.update_state, 10)
        self.setpoint_subscriber = self.create_subscription(PoseStamped, "/target", self.set_setpoint, 10)
        
        #Publisher
        self.cmd_publisher = self.create_publisher(WheelsCmdStamped, '/wheels_cmd', 10)

        control_period = 1/30

        #Controllers
        self.pose_controller = PID(Kp=0.175, Ki=0.0, Kd=0.0)
        self.omega_controller = PID(Kp=12.5, Ki=0.0, Kd=0.0)

        self.pose_controller.sample_time = control_period
        self.omega_controller.sample_time = control_period

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.xt = 0.0
        self.yt = 0.0
        self.thetat = 0.0

        self.pose_reached = False
        self.orientation_reached = False

        self.dist_tol = 0.05  # meters
        self.rot_tol = 0.026   # radians

        self.timer = self.create_timer(control_period, self.calculate_angular_control)

        self.min_speed = 0.10
        self.track_width = 0.14

        self.left_val = 0.0
        self.right_val = 0.0

    def update_state(self, msg: PoseWithCovarianceStamped):
        # PoseWithCovarianceStamped has .pose.pose similar to Odometry
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # project: project orientation.z is being used by this project as a simple yaw value
        self.theta = msg.pose.pose.orientation.z

    def set_setpoint(self, msg: PoseStamped):
        self.pose_controller.setpoint = 0 #msg.pose.position.x
        self.xt = msg.pose.position.x
        self.yt = msg.pose.position.y
        self.thetat = msg.pose.orientation.z
        self.omega_controller.setpoint = msg.pose.orientation.z

    def decompose_omega(self, omega: float) -> tuple[float, float]:
        # Convention: positive omega yields positive right wheel velocity and negative left wheel velocity for counter-clockwise rotation
        vel_left = -omega * self.track_width/2
        vel_right = omega * self.track_width/2
        print(f"Decomposed omega {omega:.4f} into vel_left: {vel_left:.4f}, vel_right: {vel_right:.4f}")
        return (vel_left, vel_right)

    def wrap_angle(self, ang: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return (ang + math.pi) % (2 * math.pi) - math.pi
    
    def calculate_angular_control(self):
        angle_err = self.wrap_angle(self.thetat - self.theta)
        omega = self.omega_controller(self.theta)
        vel_left, vel_right = self.decompose_omega(omega)

        if abs(angle_err) < self.rot_tol:
            vel_left = 0.0
            vel_right = 0.0
            self.orientation_reached = True
            self.omega_controller.reset()

        print("----")
        print(f"Angle error to target: {angle_err:.4f}")
        print(f"Current Orientation: {self.theta:.3f}, Target Orientation: {self.thetat:.3f}")
        print(f"Computed omega: {omega:.4f}, vel_left: {vel_left:.4f}, vel_right: {vel_right:.4f}")
        print("----")

        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.header.stamp = self.get_clock().now().to_msg()
        wheels_cmd.vel_left = vel_left
        wheels_cmd.vel_right = vel_right

        self.cmd_publisher.publish(wheels_cmd)


    def calculate_linear_control(self):
        # self.cmd_publisher.publish(wheel_cmd)

        dx = self.xt - self.x
        dy = self.yt - self.y

        # signed distance along the robot's forward heading (positive ahead)
        forward_dist = dx * math.cos(self.theta) + dy * math.sin(self.theta)

        # compute bearing to target and angle error (useful to ensure heading alignment)
        angle_to_target = math.atan2(dy, dx)
        angle_error = self.wrap_angle(angle_to_target - self.theta)

        print(f"Forward distance to target: {forward_dist:.4f}, angle_error: {angle_error:.4f}")
        print(f"Current Position: ({self.x:.3f}, {self.y:.3f}), Target Position: ({self.xt:.3f}, {self.yt:.3f}), theta: {self.theta:.3f}")

        # If robot is not aligned with the target, hold linear motion until rotation controller aligns it.
        angle_threshold = 1  # radians (~7 degrees)
        if abs(angle_error) > angle_threshold:
            target_velo = 0.0
        else:
            # PID setpoint is 0 (we want forward_dist -> 0)
            target_velo = self.pose_controller(forward_dist)

            # enforce a minimum speed when non-zero (preserve sign)
            if target_velo != 0.0 and abs(target_velo) < self.min_speed:
                target_velo = self.min_speed * (1 if target_velo > 0 else -1)

            # small distance tolerance -> stop
            if abs(forward_dist) < self.dist_tol:
                target_velo = 0.0
                self.pose_reached = True
                self.pose_controller.reset()

        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.header.stamp = self.get_clock().now().to_msg()
        # keep the previous inversion (negate) so sign behavior remains as before
        wheels_cmd.vel_left = -target_velo
        wheels_cmd.vel_right = -target_velo

        self.cmd_publisher.publish(wheels_cmd)
        
def main():
    rclpy.init()
    
    navigation_controller_node=Navigation_Controller_Node()
    
    rclpy.spin(navigation_controller_node)
    
    navigation_controller_node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
