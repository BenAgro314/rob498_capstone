import rospy
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose

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