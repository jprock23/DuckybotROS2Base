from cv2 import normalize
import rclpy
from geometry_msgs.msg import Point, Pose, Transform, TransformStamped, Vector3
import numpy as np
from builtin_interfaces.msg import Duration
from math import floor, sqrt
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from std_msgs.msg import ColorRGBA, Header
from map_msgs.msg import OccupancyGridUpdate
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from interfaces.msg import TagPoseArray

class Map_Generator_Node(Node):

    def __init__(self, node_name: str = "map_generator_node"):
        super().__init__(node_name)
        ##ASSUMES TAGS ARE IN UPPER RIGHT CORNER OF BOX WITH THE TOP BEING THE SHORTEST SIDE
        init_time_msg = self.get_clock().now().to_msg()
        # constants
        self._map_height = 1.82 * 2
        self._map_width = 1.82 * 2
        self._resolution = 0.01
        self._n_grid_rows = self.normalize(self._map_height)
        self._n_grid_cols = self.normalize(self._map_width)
        self._box_width = 0.155
        self._box_height = 0.23
        self._box_flap = 0.076
        self._marker_size = 0.1397
        # T_(map -> ceil_camera)
        tf_map_to_camera = TransformStamped()
        tf_map_to_camera._header._stamp = init_time_msg
        tf_map_to_camera._header._frame_id = "map"
        tf_map_to_camera._child_frame_id = "ceil_camera"
        tf_map_to_camera._transform._translation = Vector3(
            x=(0.5 * self._map_width), y=(0.5 * self._map_height), z=(-1.8)
        )
        StaticTransformBroadcaster(self).sendTransform(tf_map_to_camera) # no need to save because only doing this once
        # grid
        self._map = np.full(shape=(self._n_grid_rows, self._n_grid_cols), fill_value=0.5)
        # first map msg
        first_map_msg = OccupancyGrid()
        first_map_msg._header._stamp = init_time_msg
        first_map_msg._header._frame_id = "map"
        first_map_msg._info._map_load_time = init_time_msg
        first_map_msg._info._resolution = self._resolution
        first_map_msg._info._height = self._n_grid_rows
        first_map_msg._info._width = self._n_grid_cols
        first_map_msg._data = [int(100 * val) for val in self._map.flatten()]
        self.create_publisher(OccupancyGrid, '/map', 10).publish(first_map_msg) # no need to save because only doing this once
        # subscriber
        self._tag_sub = self.create_subscription(TagPoseArray, '/tag_poses', self.update_map, 10)
        # publishers
        self._map_update_pub = self.create_publisher(OccupancyGridUpdate, '/map_updates', 10)
        self._box_pts_markers_pub = self.create_publisher(MarkerArray, '/box_pts_markers', 10)
        # listeners
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
         
    def corners_in_map_frame(self, tag_to_map_tf: Transform) -> tuple:
        # corner points in the tag's frame, same ordering as corners in cv2.aruco
        r = sqrt(self._box_width**2 + self._box_flap**2)
        corners_in_tag_frame = (
            (-0.5 * self._marker_size, 0.5 * self._marker_size, 0.0),                                         # upper left
            (r - 0.5 * self._marker_size, 0.5 * self._marker_size, 0.0),                        # upper right
            (r - 0.5 * self._marker_size, -self._box_height + 0.5 * self._marker_size, 0.0),    # lower right
            (-0.5 * self._marker_size, -self._box_height + 0.5 * self._marker_size, 0.0)                      # lower left
        )
        # tag to map as a homogenous transformation matrix
        tag_to_map_rmtx = Rotation.from_quat([
            tag_to_map_tf._rotation._x,
            tag_to_map_tf._rotation._y,
            tag_to_map_tf._rotation._z,
            tag_to_map_tf._rotation._w
        ]).as_matrix() # 3x3 rotation matrix
        tag_to_map_tvec = [
            tag_to_map_tf._translation._x,
            tag_to_map_tf._translation._y,
            tag_to_map_tf._translation._z
        ] # 3x1 translation vector
        tag_to_map_tf_mtx = np.eye(N=4) # 4x4 homogenous transformation matrix
        tag_to_map_tf_mtx[:3, :3] = tag_to_map_rmtx
        tag_to_map_tf_mtx[:3, 3] = tag_to_map_tvec
        # transform each corner point to map's frame
        corners_in_camera_frame = []
        for tag_pt in corners_in_tag_frame:
            # transform point from cartesian coordinate to homogenous coordinate
            tag_pt_homogeneous = np.full(shape=(4, 1), fill_value=1.0) # 4x1 homogenous point
            tag_pt_homogeneous[:3, 0] = tag_pt
            # doing the transform
            transformed_homogeneous_pt = np.matmul(tag_to_map_tf_mtx, tag_pt_homogeneous).flatten()
            # recover from homogenous coordinate to cartesian coordinate
            corners_in_camera_frame.append((
                transformed_homogeneous_pt[0] / float(transformed_homogeneous_pt[3]),
                transformed_homogeneous_pt[1] / float(transformed_homogeneous_pt[3]),
                transformed_homogeneous_pt[2] / float(transformed_homogeneous_pt[3])
            ))
        return tuple(corners_in_camera_frame)

    def normalize(self, val: float):
        # real value -> grid index, does not check for index validity
        return floor(val / self._resolution)

    def update_map(self, msg: TagPoseArray):
        current_time_msg = self.get_clock().now().to_msg()
        markers = MarkerArray()
        # obtain box corners
        box_corners = []
        try:
            for tag_id in msg._ids:
                #Tag 0 is on the robot so should not be used to construct a map
                if tag_id == 0:
                    continue
                # grab transform
                tag_to_map_tf: Transform = self._tf_buffer.lookup_transform(
                    "map",
                    "tag_{}".format(tag_id),
                    rclpy.time.Time()).transform
                # calculate corner points in the camera's frame
                corners_in_camera_frame = self.corners_in_map_frame(tag_to_map_tf)
                print(f'corners:: {corners_in_camera_frame}')
                print(f'id:: {tag_id}\n')
                box_corners.append(corners_in_camera_frame)
                # marker
                marker = Marker()
                marker._header._stamp = current_time_msg
                marker._header._frame_id = "map"
                marker._id = tag_id
                marker._type = int(4)
                marker._action = int(0)
                marker._scale = Vector3(x=float(0.01))
                marker._color = ColorRGBA(
                    r=float(0.0), 
                    g=float(1.0), 
                    b=float(0.0), 
                    a=float(1.0)
                )
                for pt in corners_in_camera_frame:
                    marker._points.append(Point(
                        x=float(pt[0]),
                        y=float(pt[1]),
                        z=float(pt[2])
                    ))
                marker._points.append(Point(
                    x=float(corners_in_camera_frame[0][0]),
                    y=float(corners_in_camera_frame[0][1]),
                    z=float(corners_in_camera_frame[0][2]),
                ))
                marker._lifetime = Duration(
                    sec=int(0),
                    nanosec=int(5e+8)
                )
                markers._markers.append(marker)
        except:
            pass
        # publish markers
        self._box_pts_markers_pub.publish(markers)
        # make new grid with map corners
        map_given_current_obs: np.ndarray = np.full(shape=self._map.shape, fill_value=0.25) # assume no box
        for box in box_corners:
            '''
                - idea: 
                    - represent each cell in the grid as a point
                    - create a convex hull using box corners
                    - for each cell, create a convex hull using 
                        (box corners + cell point)
                    - if the (new convex hull - original convex hull == 0), then
                      adding the cell point does not change the convex hull, 
                      meaning the cell point is inside the box
                - optimization strategy:  only check hypothetical cells close to the box
                - addition note: need to check if hypothetical cells are in the map
            '''
            box_2d = [(pt[0], pt[1]) for pt in box]
            hull_vertices_from_box = set(ConvexHull(box_2d).vertices)
            # optimization strategy: only check cells close to the box
            for x_val in np.arange(
                min(box[0][0], box[1][0], box[2][0], box[3][0]),
                max(box[0][0], box[1][0], box[2][0], box[3][0]) + self._resolution,
                self._resolution
            ):
                for y_val in np.arange(
                    min(box[0][1], box[1][1], box[2][1], box[3][1]),
                    max(box[0][1], box[1][1], box[2][1], box[3][1]) + self._resolution,
                    self._resolution
                ):
                    
                    cell_row_index = self.normalize(y_val)
                    cell_col_index = self.normalize(x_val)
                    if (
                        ((0 <= cell_row_index) and (cell_row_index < self._n_grid_rows))
                        and ((0 <= cell_col_index) and (cell_col_index < self._n_grid_cols))
                    ):
                        new_hull_pts = box_2d.copy()
                        new_hull_pts.append((x_val, y_val))
                        new_hull_vertices = set(ConvexHull(new_hull_pts).vertices)
                        if (len(new_hull_vertices.difference(hull_vertices_from_box)) == 0): # set difference order matters
                            map_given_current_obs[cell_row_index][cell_col_index] = 0.75
        # updating occupancy grid probabilities, see https://jokane.net/x52/13-slam/
        for row_index in range(self._n_grid_rows):
            for col_index in range(self._n_grid_cols):
                pr_cell_occupied_given_current_obs = map_given_current_obs[row_index][col_index]
                pr_cell_free_given_current_obs = 1.0 - pr_cell_occupied_given_current_obs
                pr_cell_occupied_given_previous_obs = self._map[row_index][col_index]
                pr_cell_free_given_previous_obs = 1.0 - pr_cell_occupied_given_previous_obs
                pr_cell_occupied = (
                    1.0 + (
                        (pr_cell_free_given_current_obs * pr_cell_free_given_previous_obs) 
                        / (pr_cell_occupied_given_current_obs * pr_cell_occupied_given_previous_obs)
                    )
                ) ** (-1)
                self._map[row_index][col_index] = pr_cell_occupied
        # update map
        grid_update_msg = OccupancyGridUpdate()
        grid_update_msg.header = Header()
        grid_update_msg.header.frame_id = "map"
        grid_update_msg.header.stamp = current_time_msg
        grid_update_msg._height = self._n_grid_rows
        grid_update_msg._width = self._n_grid_cols
        grid_update_msg.data = [int(100 * val) for val in self._map.flatten()]
        self._map_update_pub.publish(grid_update_msg)

def main():
    rclpy.init()

    map_generator_node = Map_Generator_Node()

    rclpy.spin(map_generator_node)

    map_generator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()