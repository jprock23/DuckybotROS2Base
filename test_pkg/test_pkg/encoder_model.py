import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
from interfaces.msg import WheelsCmdStamped, WheelEncoderStamped

class Encoder_Model_Node(Node):

    def __init__(self, node_name='encoder_model_node'):
        super().__init__(node_name)

        #subscribers

        #publishers

        # listeners
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
    def pub_ticks(self):
        pass


def main():
    rclpy.init()

    encoder_model_node = Encoder_Model_Node()

    rclpy.spin(encoder_model_node)

    encoder_model_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()