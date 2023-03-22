import rospy
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Quaternion, Twist
import math
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import tf.transformations as tf_transform

class Colors:
    """
    This class is used to print text to the terminal in color. 
    Basic Usage:
    print(f"{Colors.COLOR}my text{Colors.RESET}")
    """
    RED   = "\033[1;31m"  
    BLUE  = "\033[1;34m"
    CYAN  = "\033[1;36m"
    GREEN = "\033[0;32m"
    RESET = "\033[0;0m"
    BOLD    = "\033[;1m"
    REVERSE = "\033[;7m"

def pose_to_numpy(pose):
    # Extract position and orientation from the Pose message
    position = pose.position
    orientation = pose.orientation

    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])

    # Create the transformation matrix
    transformation_matrix = np.identity(4)
    transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
    transformation_matrix[:3, 3] = [position.x, position.y, position.z]

    return transformation_matrix


def transform_stamped_to_numpy(transform_stamped: TransformStamped):
    # Extract translation and rotation from the TransformStamped message
    translation = transform_stamped.transform.translation
    rotation = transform_stamped.transform.rotation

    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

    # Create the transformation matrix
    transformation_matrix = np.identity(4)
    transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
    transformation_matrix[:3, 3] = [translation.x, translation.y, translation.z]

    return transformation_matrix

def pose_stamped_to_numpy(pose_stamped: PoseStamped):
    # Extract position and orientation from the PoseStamped message
    position = pose_stamped.pose.position
    orientation = pose_stamped.pose.orientation

    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])

    # Create the transformation matrix
    transformation_matrix = np.identity(4)
    transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
    transformation_matrix[:3, 3] = [position.x, position.y, position.z]

    return transformation_matrix

def numpy_to_pose_stamped(transformation_matrix, frame_id="parent_frame"):
    # Check if the input is a 4x4 matrix
    if transformation_matrix.shape != (4, 4):
        raise ValueError("The input must be a 4x4 NumPy array")

    # Extract position and rotation components
    position = transformation_matrix[:3, 3]
    quaternion = quaternion_from_matrix(transformation_matrix)

    # Create a PoseStamped message
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = frame_id

    pose_stamped.pose.position.x = position[0]
    pose_stamped.pose.position.y = position[1]
    pose_stamped.pose.position.z = position[2]

    pose_stamped.pose.orientation.x = quaternion[0]
    pose_stamped.pose.orientation.y = quaternion[1]
    pose_stamped.pose.orientation.z = quaternion[2]
    pose_stamped.pose.orientation.w = quaternion[3]

    return pose_stamped

def pose_to_transform_stamped(pose: Pose, frame_id="parent_frame", child_frame_id="child_frame"):
    # Extract position and orientation from the Pose message
    position = pose.position
    orientation = pose.orientation

    # Create a TransformStamped message
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = frame_id
    transform_stamped.child_frame_id = child_frame_id

    transform_stamped.transform.translation.x = position.x
    transform_stamped.transform.translation.y = position.y
    transform_stamped.transform.translation.z = position.z

    transform_stamped.transform.rotation.x = orientation.x
    transform_stamped.transform.rotation.y = orientation.y
    transform_stamped.transform.rotation.z = orientation.z
    transform_stamped.transform.rotation.w = orientation.w

    return transform_stamped

def pose_stamped_to_transform_stamped(pose_stamped: PoseStamped, parent_frame_id=None, child_frame_id=None):
    """
    Convert a PoseStamped message to a TransformStamped message.
    
    Args:
        pose_stamped (PoseStamped): The PoseStamped message to be converted.
        parent_frame_id (str, optional): The parent frame ID for the TransformStamped.
                                         If None, it will use the same frame_id as the input PoseStamped.
    
    Returns:
        TransformStamped: The converted TransformStamped message.
    """
    transform_stamped = TransformStamped()

    # Fill in the header information
    transform_stamped.header.stamp = pose_stamped.header.stamp
    transform_stamped.header.frame_id = parent_frame_id if parent_frame_id else pose_stamped.header.frame_id
    transform_stamped.child_frame_id = pose_stamped.header.frame_id
    if child_frame_id is not None:
        transform_stamped.child_frame_id = child_frame_id

    # Set the translation
    transform_stamped.transform.translation.x = pose_stamped.pose.position.x
    transform_stamped.transform.translation.y = pose_stamped.pose.position.y
    transform_stamped.transform.translation.z = pose_stamped.pose.position.z

    # Set the rotation
    transform_stamped.transform.rotation.x = pose_stamped.pose.orientation.x
    transform_stamped.transform.rotation.y = pose_stamped.pose.orientation.y
    transform_stamped.transform.rotation.z = pose_stamped.pose.orientation.z
    transform_stamped.transform.rotation.w = pose_stamped.pose.orientation.w

    return transform_stamped

def get_config_from_pose_stamped(pose_stamped):
    """
    Compute the x, y, z, yaw from a PoseStamped message.
    
    Args:
        pose_stamped (PoseStamped): The PoseStamped message containing the orientation.
    
    Returns:
        np.array([float, float, float, float])
    """
    # Extract the orientation quaternion
    q = pose_stamped.pose.orientation
    x = pose_stamped.pose.position.x
    y = pose_stamped.pose.position.y
    z = pose_stamped.pose.position.z

    # Convert the quaternion to Euler angles (roll, pitch, yaw)
    _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

    return np.array([x, y, z, yaw])

def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).

    Args:
        x (float): The x component of the quaternion.
        y (float): The y component of the quaternion.
        z (float): The z component of the quaternion.
        w (float): The w component of the quaternion.

    Returns:
        tuple: A tuple containing the Euler angles (roll, pitch, yaw) in radians.
    """
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)  # Clamp value to the range [-1, 1]
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

def are_angles_close(angle1, angle2, tol):
    """
    Check if two angles in radians are within a specified tolerance.

    Args:
        angle1 (float): The first angle in radians.
        angle2 (float): The second angle in radians.
        tol (float): The tolerance value in radians.

    Returns:
        bool: True if the angles are within the tolerance, False otherwise.
    """
    # Calculate the difference between the angles, considering the wrap-around at 2*pi
    diff = shortest_signed_angle(angle1, angle2)

    # Check if the difference is within the specified tolerance
    return abs(diff) <= tol


def shortest_signed_angle(a1, a2):
    """
    Calculate the shortest signed angle to rotate from angle a1 to angle a2, both expressed in radians.

    Args:
        a1 (float): The starting angle in radians.
        a2 (float): The ending angle in radians.

    Returns:
        float: The shortest signed angle to rotate from a1 to a2 in radians.
    """
    # Compute the difference between the angles, considering the wrap-around at 2*pi
    diff = math.atan2(math.sin(a2 - a1), math.cos(a2 - a1))

    return diff

def config_to_transformation_matrix(x, y, z, yaw):
    """
    Creates a 4x4 transformation matrix from an (x, y, z, yaw) tuple.
    """
    T = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, x],
        [np.sin(yaw), np.cos(yaw), 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

    return T

def se2_pose_list_to_path(pose_list, ref_frame):
    # convert a list of poses to a path
    path = Path()
    path.header.frame_id = ref_frame
    for pose in pose_list:
        ros_pose = PoseStamped()
        ros_pose.pose.position.x = pose[0]
        ros_pose.pose.position.y = pose[1]
        ros_pose.pose.orientation = ros_quat_from_euler([0, 0, pose[2]])
        ros_pose.header.frame_id = ref_frame
        path.poses.append(ros_pose)
    return path

def ros_quat_from_euler(e):
    # get a ROS XYZW quaternion from an SXYZ euler
    np_q = tf_conversions.transformations.quaternion_from_euler(*e)
    return ros_q_from_np_q(np_q)

def ros_q_from_np_q(np_q):
    q = Quaternion()
    q.x = np_q[0]; q.y = np_q[1]; q.z = np_q[2]; q.w = np_q[3]
    return q

def make_sphere_marker(x: float, y: float, z: float, r: float):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = "sphere"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.scale.x = r
    marker.scale.y = r
    marker.scale.z = r
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.5
    marker.lifetime = rospy.Duration()

    marker.header.stamp = rospy.Time.now()
    return marker

def transform_twist(twist_b: Twist, t_a_b: np.array):
    # Extract rotation matrix r_a_b and translation vector p from the transformation matrix
    r_a_b = t_a_b[:3, :3]
    p = t_a_b[:3, 3]

    # Compute the skew-symmetric matrix from the translation vector p
    p_x = np.array([
        [0, -p[2], p[1]],
        [p[2], 0, -p[0]],
        [-p[1], p[0], 0]
    ])

    # Compute the adjoint transformation matrix
    ad_t_a_b = np.zeros((6, 6))
    ad_t_a_b[:3, :3] = r_a_b
    ad_t_a_b[:3, 3:] = p_x @ r_a_b
    ad_t_a_b[3:, 3:] = r_a_b
    #ad_t_a_b = np.block([
    #    [r_a_b, p_x @ r_a_b]
    #    [np.zeros((3, 3)), r_a_b],
    #])

    # Convert Twist command to a 6x1 numpy array
    twist_b_np = np.array([
        [twist_b.linear.x],
        [twist_b.linear.y],
        [twist_b.linear.z],
        [twist_b.angular.x],
        [twist_b.angular.y],
        [twist_b.angular.z]
    ])

    # Transform the Twist command
    twist_a_np = ad_t_a_b @ twist_b_np

    # Convert the transformed numpy array back to a Twist message
    twist_a = Twist()
    twist_a.linear.x = twist_a_np[0, 0]
    twist_a.linear.y = twist_a_np[1, 0]
    twist_a.linear.z = twist_a_np[2, 0]
    twist_a.angular.x = twist_a_np[3, 0]
    twist_a.angular.y = twist_a_np[4, 0]
    twist_a.angular.z = twist_a_np[5, 0]

    return twist_b

def slerp_pose(pose1: Pose, pose2: Pose, timestamp1: rospy.Time, timestamp2: rospy.Time, target_timestamp: rospy.Time, frame_id: str):
    # Calculate the interpolation factor
    t = (target_timestamp - timestamp1).to_sec() / (timestamp2 - timestamp1).to_sec()
    print(t)

    # Interpolate positions using linear interpolation
    position1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
    position2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])
    interp_position = (1 - t) * position1 + t * position2

    # Extract quaternions from the poses
    q1 = np.array([pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w])
    q2 = np.array([pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w])

    # Interpolate quaternions using SLERP
    interp_quaternion = tf_transform.quaternion_slerp(q1, q2, t)

    # Construct the interpolated PoseStamped message
    interp_pose = PoseStamped()
    interp_pose.header.stamp = rospy.Time.from_sec(target_timestamp.to_sec())
    interp_pose.header.frame_id = frame_id
    interp_pose.pose.position.x = interp_position[0]
    interp_pose.pose.position.y = interp_position[1]
    interp_pose.pose.position.z = interp_position[2]
    interp_pose.pose.orientation.x = interp_quaternion[0]
    interp_pose.pose.orientation.y = interp_quaternion[1]
    interp_pose.pose.orientation.z = interp_quaternion[2]
    interp_pose.pose.orientation.w = interp_quaternion[3]

    return interp_pose

def transform_stamped_to_pose_stamped(transform_stamped: TransformStamped):
    """
    Converts a geometry_msgs/TransformStamped into a geometry_msgs/PoseStamped.

    Args:
    transform_stamped (geometry_msgs/TransformStamped): The input TransformStamped to be converted.

    Returns:
    pose_stamped (geometry_msgs/PoseStamped): The resulting PoseStamped from the given TransformStamped.
    """
    pose_stamped = PoseStamped()

    # Copy the header from the TransformStamped to the PoseStamped
    pose_stamped.header = transform_stamped.header

    # Copy the position from the TransformStamped to the PoseStamped
    pose_stamped.pose.position.x = transform_stamped.transform.translation.x
    pose_stamped.pose.position.y = transform_stamped.transform.translation.y
    pose_stamped.pose.position.z = transform_stamped.transform.translation.z

    # Copy the orientation from the TransformStamped to the PoseStamped
    pose_stamped.pose.orientation.x = transform_stamped.transform.rotation.x
    pose_stamped.pose.orientation.y = transform_stamped.transform.rotation.y
    pose_stamped.pose.orientation.z = transform_stamped.transform.rotation.z
    pose_stamped.pose.orientation.w = transform_stamped.transform.rotation.w

    return pose_stamped