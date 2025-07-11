# a collection of geometry related functions
# ================================================
import geometry_msgs.msg as ros_geom
import numpy
import transformations
# ================================================
# converting between Ros format and TF format
def quat_ros_to_tf(quat_ros: ros_geom.Quaternion) -> numpy.ndarray:
    quat_tf = numpy.array([quat_ros._x, quat_ros._y, quat_ros._z, quat_ros._w])
    return transformations.unit_vector(quat_tf)

def quat_tf_to_ros(quat_tf:numpy.ndarray) -> ros_geom.Quaternion:
    quat_tf_normalized = transformations.unit_vector(quat_tf)
    quat_ros = ros_geom.Quaternion()
    quat_ros._x = float(quat_tf_normalized[0])
    quat_ros._y = float(quat_tf_normalized[1])
    quat_ros._z = float(quat_tf_normalized[2])
    quat_ros._w = float(quat_tf_normalized[3])
    return quat_ros
# ================================================
# converting between euler and quaternion
def euler_to_quat_ros(roll=0.0, pitch=0.0, yaw=0.0) -> ros_geom.Quaternion:
    quat_tf_normalized = transformations.unit_vector(transformations.quaternion_from_euler(roll, pitch, yaw))
    return quat_tf_to_ros(quat_tf_normalized)

def quat_ros_to_euler(quat: ros_geom.Quaternion) -> numpy.ndarray:
    return transformations.euler_from_quaternion(quat_ros_to_tf(quat))
# ================================================
# making Ros types
def mk_pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> ros_geom.Pose:
    pose = ros_geom.Pose()
    pose._position._x = float(x)
    pose._position._y = float(y)
    pose._position._z = float(z)
    pose._orientation = euler_to_quat_ros(roll, pitch, yaw)
    return pose
# ================================================
# quaternion calculations with Ros
def quat_ros_multiply(quat_ros_a:ros_geom.Quaternion, quat_ros_b:ros_geom.Quaternion) -> ros_geom.Quaternion:
    quat_tf_a = quat_ros_to_tf(quat_ros_a)
    quat_tf_b = quat_ros_to_tf(quat_ros_b)
    quatOutputTFNormalized = transformations.quaternion_multiply(quat_tf_a, quat_tf_b)
    return quat_tf_to_ros(quatOutputTFNormalized)

def calc_relative_rot(quat_ros_a:ros_geom.Quaternion, quat_ros_b:ros_geom.Quaternion) -> ros_geom.Quaternion:
    # Given 2 quaternions, calculate the relative rotation from quat_ros_a to quat_ros_b.
    # Output a quaternion representing the relative rotation
    # See Relative Rotation from https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html
    quat_tf_a = quat_ros_to_tf(quat_ros_a)
    quat_tf_b = quat_ros_to_tf(quat_ros_b)
    quatR = transformations.quaternion_multiply(quat_tf_b, transformations.quaternion_inverse(quat_tf_a))
    return quat_tf_to_ros(quatR)
# ================================================
if __name__ == "__main__":
    pass
