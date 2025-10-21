import rclpy
from rclpy.node import Node
import simple_pid

from geometry_msgs.msg import PoseStamped
from interfaces.msg import WheelsCmdStamped

class Navigation_Controller_Node(Node):
    
    def __init__(self):
        super().__init__("navigation_controller_node")
        
        #Subscirbers
        # self.target_pose_subscriber = self.create_subscription(PoseStamped, "/target_pose", self.set_setpoint, 10)
        self.input_sub = self.create_subscription(WheelsCmdStamped, "/input", self.update, 10)
        
        #Publisher
        self.cmd_publisher = self.create_publisher(WheelsCmdStamped, '/wheels_cmd', 10)

        self.timer = self.create_timer(1/30, self.pub_cmd)

        self.left_val = 0.0
        self.right_val = 0.0

    def set_setpoint(self, msg: PoseStamped):
        pass

    def update(self, msg:WheelsCmdStamped):
        self.left_val = msg.vel_left
        self.right_val = msg.vel_right

    def pub_cmd(self):
        out_msg = WheelsCmdStamped()

        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.vel_left = self.left_val
        out_msg.vel_right = self.right_val

        self.cmd_publisher.publish(out_msg)
        
def main():
    rclpy.init()
    
    navigation_controller_node =Navigation_Controller_Node()
    
    rclpy.spin(navigation_controller_node)
    
    navigation_controller_node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
