#!/usr/bin/env python3

from tf.transformations import quaternion_matrix
import numpy as np
# from offboard_py.scripts.utils import get_config_from_pose_stamped, transform_stamped_to_numpy, pose_stamped_to_numpy, numpy_to_pose_stamped
# from threading import Sebasehore, Lock
# import rospy
# from copy import deepcopy
# import numpy as np
# from geometry_msgs.msg import PoseStamped, TransformStamped
# from mavros_msgs.msg import State

np.random.seed(0)

# Frame names
# Map is frame PX4 sets at the start of intilization
# Base is the frame of where the PX4 currently is
# Dots is the frame of the vicon dots
# Global is the base frame of the vicon system

# C_ba transforms points in frame a to frame b


def point_cloud_alignment(point_set_Fa: np.ndarray, point_set_Fb: np.ndarray):
    '''
    Description-----
    Computes the rigid transformation matrix from frame A to B

    p_b = C_ba(p_a + r_offset)+ r_ba
    p_b = C_ba * p_a + (C_ba * r_offset + r_ba)

    Input-----
    point_set_Fa = 3xN numpy array where col is form (x,y,z)
    point_set_Fb = 3xN numpy array where col is form (x,y,z)

    Output-----
    T_ba = 4x4 numpy array [[C_ba,r_ba],[0,1]]
    '''

    # Check inputs are right

    # pose array 3 x points each row (x,y,z)

    if not isinstance(point_set_Fa, np.ndarray) or not isinstance(point_set_Fb, np.ndarray):
        raise ValueError(
            "The inputs must be  3xN NumPy arrays where each row is [x,y,z]")

    elif point_set_Fa.shape[0] != 3 or point_set_Fb.shape[0] != 3:
        raise ValueError(
            "The inputs must be  3xN NumPy arrays where each row is [x,y,z]")

    elif point_set_Fa.shape != point_set_Fb.shape:
        raise ValueError("Arrays must be same shape")

    num_points = point_set_Fa.shape[1]

    point_set_Fa = point_set_Fa.astype(np.float64)
    point_set_Fb = point_set_Fb.astype(np.float64)

    # Get centroids form 3x1
    p_Fa = (np.sum(point_set_Fa, axis=1)/num_points)[:, None]
    p_Fb = (np.sum(point_set_Fb, axis=1)/num_points)[:, None]

    # Outer product matrix
    W_ba = (point_set_Fb - p_Fb) @ (point_set_Fa - p_Fa).T
    W_ba /= num_points

    V, _, UT = np.linalg.svd(W_ba)

    C_ba = np.eye(3, 3)
    C_ba[2, 2] = np.linalg.det(UT.T) * np.linalg.det(V)
    C_ba = V@C_ba@UT

    print("Point cloud alignment completed")

    ra_ba = (-(C_ba.T)@p_Fb)+p_Fa

    T_ba = np.eye(4, 4)
    T_ba[:3, :3] = C_ba
    T_ba[:3, 3] = np.squeeze(C_ba@-ra_ba)
    return T_ba




if __name__ == "__main__":
    # Test 1
    a = np.array([[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1]]).T

    sqrt_t = np.sqrt(2)
    b = np.array([[1/sqrt_t, 1/sqrt_t, -1], [-1/sqrt_t, 1/sqrt_t, -1],
                 [-1/sqrt_t, -1/sqrt_t, -1], [1/sqrt_t, -1/sqrt_t, -1]]).T

    T_guess = point_cloud_alignment(a, b)
    C_real = [[1/sqrt_t, -1/sqrt_t, 0], [1/sqrt_t, 1/sqrt_t, 0], [0, 0, 1]]
    print(np.round(T_guess[:3, :3]-C_real, 3)) # Check rotation, should all be 0

    # Test 2
    pts = 5
    a = np.random.rand(3, pts) * 10
    a_offset = a +np.random.randint(5, size=(3, 1))  # Random offset

    # Convert quaternion to rotation matrix
    x, y, z, w = np.random.rand(4) * 10
    C_ba = quaternion_matrix([x, y, z, w])[:3, :3]

    offset2 = np.random.randint(5, size=(3, 1))
    b = C_ba@(a_offset+offset2)

    T_guess = point_cloud_alignment(a, b)
    print(np.round(T_guess[:3, :3]-C_ba, 2)) # Check rotation, should all be 0

    a = np.vstack((a, np.ones((1, pts))))
    b_guess = (T_guess@a)
    b_guess = b_guess[:3, :]
    print(np.round(b_guess-b)) # Check transfromation all, should all be 0




'''
class base_to_dots():

    def __init__(self, num_points=10):
        self.map_pose_sub = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, callback=self.base_cb)
        self.global_sub = rospy.Subscriber(
            "/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, self.vicon_cb)
        self.state_sub = rospy.Subscriber(
            "mavros/state", State, callback=self.state_cb)

        self.setpoint_position_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10)

        self.current_state = State()

        self.num_points = num_points

        # The base frame as seen in map frame
        self.current_p_base_Fm = np.zeros(1, 3)
    
        # The dots frame as seen in global frame
        self.current_dots_pose_Fg = np.zeros(1, 3)

        # Sizes 3xpoints each row (x,y,z)
        self.base_pose_Fm_array = np.zeros(num_points, 3)
        self.dots_pose_Fg_array = np.zeros(num_points, 3)

        # Waypoints expressed in map frame
        self.waypoints_queue_Fm = []

        for i in range(num_points):
            pose_Fm = np.eye(4)

            # xy within 2m by 2m plane, units meters
            pose_Fm[0, 3] = random.randint(0, 200)/100.0
            pose_Fm[1, 3] = random.randint(0, 200)/100.0
            # z within 0.6m - 1m ,
            pose_Fm[2, 3] = random.randint(60, 100)/100.0

            pose_Fm = numpy_to_pose_stamped(pose_Fm, frame_id="map_frame")
            self.waypoints_queue_Fm.append(pose_Fm)

        # Waypoints expressed in map frame
        self.current_waypoint_Fm = self.waypoints_queue_Fm.pop()

        self.tf_base_to_dots = np.eye(4, 4)

    def state_cb(self, msg: State):
        self.current_state = msg

    def base_cb(self, base_point: PoseStamped):
        self.current_p_base_Fm = pose_stamped_to_numpy(base_point)

    def vicon_cb(self, vicon_point: PoseStamped):
        self.current_dots_pose_Fg = transform_stamped_to_numpy(vicon_point)

    def pose_is_close(self, pose1: PoseStamped, pose2: PoseStamped):
        # TOOD: include some meaasure of rotation diff
        cfg1 = get_config_from_pose_stamped(pose1)
        cfg2 = get_config_from_pose_stamped(pose2)
        trans_is_close = np.linalg.norm(
            cfg1[:3] - cfg2[:3]) < self.waypoint_trans_ths
        # print(f"Trans is close: {trans_is_close}. Yaw is close: {yaw_is_close}")
        return trans_is_close


    def point_cloud_alignment(self):
        # pose array 3 x points each row (x,y,z)

        # Get centroids form 1x3
        p_base = np.sum(self.base_pose_Fm_array,axis = 0)/self.num_points
        p_dots = np.sum(self.dots_pose_Fg_array,axis = 0)/self.num_points

        for idx in range(self.num_points):
            w_dots_base +=(self.dots_pose_Fg_array[idx,:]-p_dots).T@(self.base_pose_Fm_array[idx,:]-p_base)

        U, S, VT = np.linalg.svd(w_dots_base, full_matrices=True)

        C_dots_base = np.eye(3,3)
        C_dots_base[3,3] = np.linalg.det(VT.T) @np.linalg.det(U)
        C_dots_base = U@C_dots_base@VT

        r_dots_base = -C_dots_base.T@p_dots + p_base

        print("Calibration completed")

        self.tf_base_to_dots[:3,:3] = C_dots_base
        self.tf_base_to_dots[3,:3] = -r_dots_base


    def calibrate(self):
        rate = rospy.Rate(20)

        print(f"Trying to connect to drone...")
        while (not rospy.is_shutdown() and not self.current_state.connected):
            rate.sleep()
        print(f"Connected!")

        while (not rospy.is_shutdown()):
            # Can move the drone by hand
            #self.setpoint_position_pub.publish(self.current_waypoint)

            if self.pose_is_close(self.current_waypoint_Fm, self.current_p_base_Fm):
                if len(self.waypoints_queue_Fm) == 0:
                    self.point_cloud_alignment()

                    # Land it
                    pose_Fm = np.eye(4)
                    pose_Fm[:3, 3] = [0, 0, 0.05]
                    pose_Fm = numpy_to_pose_stamped(pose_Fm, frame_id="map_frame")
                    self.setpoint_position_pub.publish(pose_Fm)

                idx = len(self.waypoints_queue_Fm)
                base_pose_Fm_np = pose_stamped_to_numpy(self.current_p_base_Fm)
                dots_pose_Fg_np = pose_stamped_to_numpy(self.current_dots_pose_Fg)

                # pose array 3xpoints each row (x,y,z)
                # np 4x4
                self.base_pose_Fm_array[idx, :] = base_pose_Fm_np[:3, 3]
                self.dots_pose_Fg_array[idx, :] = dots_pose_Fg_np[:3, 3]

                self.current_waypoint_Fm = self.waypoints_queue_Fm.pop()
                rate.sleep()


if __name__ == "__main__":
    rospy.init_node("px4_vicon_calibration")
    calibartion = base_to_dots()
    calibartion.calibrate()
'''
