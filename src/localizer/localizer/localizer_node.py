import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
from interfaces.msg import WheelsCmdStamped, WheelEncoderStamped

class Localizer_Node(Node):

    def __init__(self, node_name='localizer_node'):
        super().__init__(node_name)

        self.num_particles = 25
        self.particles = []
        for i in range(self.num_particles):
            p = Pose()
            p.position.x = 0.0
            p.position.y = 0.0
            p.position.z = 0.0

            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0 
            p.orientation.w = 1.0 

            self.particles.append(p)

        self.velocities = WheelsCmdStamped()

        #subscribers
        self.motor_sub = self.create_subscription(WheelsCmdStamped, '/wheels_cmd', self.update_expiration_time, 10)
        self.encoderL_sub = self.create_subscription(WheelEncoderStamped,'left_encoder_node/tick', )
        self.encoderR_sub = self.create_subscription(WheelEncoderStamped)

        #publishers

        # listeners
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def update_expiration_time(self, msg):
        self.velocities = msg

    def forward_projection(self, msg):
        expiration_time = float(self.velocities.header.stamp.sec + 1) + float(self.velocities.header.stamp.nanosec/1e9)

        if(self.get_clock().now().to_msg() < expiration_time):
            pass
        

def main():
    rclpy.init()

    localizer_node = Localizer_Node()

    rclpy.spin(localizer_node)

    localizer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()