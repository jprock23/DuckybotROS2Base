from math import ceil
import numpy as np

from interfaces.msg._tag_pose_stamped import TagPoseStamped
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Quaternion, Point, Transform
from nav_msgs.msg import OccupancyGrid

class Map_Generator_Node(Node):

    def __init__(self,):
        super().__init__('map_generator_node')
        ##ASSUMES TAGS ARE IN UPPER RIGHT CORNER OF BOX WITH THE TOP BEING THE SHORTEST SIDE

        height = 1.78
        width = 1.78
        self.resolution = 0.01

        self.norm_height = normalize(height, self.resolution)
        self.norm_width = normalize(width, self.resolution)
        print(f"height:: {self.norm_height}, width:: {self.norm_width}")

        self.origin = Pose()
        self.origin.position.x = -0.9268597160482871
        self.origin.position.y = -0.8796537877987689
        self.origin.position.z = 1.626891409883178

        self.origin.orientation.x = 0.0
        self.origin.orientation.y = 0.0
        self.origin.orientation.z = 0.0
        self.origin.orientation.w = 1.0

        box_width = 0.155
        box_height = 0.23
        self.norm_box_height = normalize(box_height, self.resolution)
        self.norm_box_width = normalize(box_width, self.resolution)
        print(f"box_height:: {self.norm_box_height}, box_width:: {self.norm_box_width}")

        self.marker_size = 0.096
        self.transform = Transform()

        self.grid = np.zeros((self.norm_height, self.norm_width), dtype=np.int8)      

        #Subscribers
        self.tag_sub = self.create_subscription(TagPoseStamped, '/camera/tag_pose', self.gen_map, 10)

        #Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.grid_pub)

        #Listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    
    def tf_cb(self, id):
        try:
            transform = self.tf_buffer.lookup_transform(
                'ceil_camera',
                f'tag_{id}',
                rclpy.time.Time()).transform
            # print(self.transform)
            return transform
        except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform tag_{id} to ceil_camera: {ex}')

    def gen_map(self, msg):
        tag_position = msg.pose.position


        print(f'frame:: {msg.tag_id}')
        transform = self.tf_cb(msg.tag_id)

        # x = int(tag_position.x)
        # y = int(tag_position.y)
        
        x = normalize(tag_position.x, self.resolution)
        y = normalize(tag_position.y, self.resolution)
        
        print(f"tag_x:: {x}, tag_y:: {y}")
        print(f"x_bound:: {x+self.norm_box_height}, tag_y:: {y-self.norm_box_width}")


        for i in range(x, x + self.norm_box_height):
            for j in range(y, y - self.norm_box_width):
                if 0 <= i < self.norm_height and 0 <= j < self.norm_width:
                    # print("test")
                    self.grid[i][j] = 1

    def grid_pub(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = 'ceil_camera'
        msg.header.stamp = self.get_clock().now().to_msg()

        print(self.grid)
        msg.data = [int(x) for x in self.grid.flatten()]

        msg.info.height = self.norm_height
        msg.info.width = self.norm_width
        msg.info.resolution = self.resolution
        msg.info.origin = self.origin

        self.map_pub.publish(msg)

def main():
    rclpy.init()

    map_generator_node = Map_Generator_Node()

    rclpy.spin(map_generator_node)

    map_generator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

def normalize(val, unit):
    return ceil(val/unit)