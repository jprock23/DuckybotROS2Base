import rclpy
from rclpy.node import Node
import simple_pid

from geometry_msgs.msg import PoseStamped

class Navigation_Controller_Node(Node):
    
    def __init__(self):
        super().__init__("navigation_controller_node")
        
        #Subscirbers
        self.target_pose_subscriber = self.create_subscription(PoseStamped, "/target_pose", self.set_setpoint, 10)
        
        
    def set_setpoint(self, msg: PoseStamped):
        pass
        
def main():
    rclpy.init()
    
    navigation_controller_node =Navigation_Controller_Node()
    
    rclpy.spin(navigation_controller_node)
    
    navigation_controller_node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
