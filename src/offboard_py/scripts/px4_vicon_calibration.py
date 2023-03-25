#!/usr/bin/env python3

import random
from offboard_py.scripts.utils import get_config_from_pose_stamped, transform_stamped_to_numpy, pose_stamped_to_numpy, numpy_to_pose_stamped
from threading import Semaphore, Lock
import rospy
from copy import deepcopy
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import State

np.random.seed(0)


class map_to_dots():

    def __init__(self, points=10):
        self.map_pose_sub = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, callback=self.map_cb)
        self.vicon_sub = rospy.Subscriber(
            "/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, self.vicon_cb)
        self.state_sub = rospy.Subscriber(
            "mavros/state", State, callback=self.state_cb)

        self.setpoint_position_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10)

        self.current_state = State()

        self.points = points

        self.current_map_pose = np.zeros(1, 3)
        self.current_dots_pose = np.zeros(1, 3)

        # Sizes 3xpoints each row (x,y,z)
        self.map_pose_array = np.zeros(points, 3)
        self.dots_pose_array = np.zeros(points, 3)

        self.waypoints_queue = []

        for i in range(points):
            pose = np.eye(4)

            # xy within 2m by 2m plane, units meters
            pose[0, 3] = random.randint(0, 200)/100.0
            pose[1, 3] = random.randint(0, 200)/100.0
            # z within 0.6m - 1m ,
            pose[2, 3] = random.randint(60, 100)/100.0

            pose = numpy_to_pose_stamped(pose, frame_id="parent_frame")
            self.waypoints_queue.append(pose)

        self.current_waypoint = self.waypoints_queue.pop()

        self.tf_map_to_dots = np.eye(4, 4)

    def state_cb(self, msg: State):
        self.current_state = msg

    def map_cb(self, map_point: PoseStamped):
        self.current_map_pose = pose_stamped_to_numpy(map_point)

    def vicon_cb(self, vicon_point: PoseStamped):
        self.current_dots_pose = transform_stamped_to_numpy(vicon_point)

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
        p_map = np.sum(self.map_pose_array,axis = 0)/self.points
        p_dots = np.sum(self.dots_pose_array,axis = 0)/self.points

        for idx in range(self.points):
            w_dots_map +=(self.dots_pose_array[idx,:]-p_dots).T@(self.map_pose_array[idx,:]-p_map)

        U, S, VT = np.linalg.svd(w_dots_map, full_matrices=True)

        C_dots_map = np.eye(3,3)
        C_dots_map[3,3] = np.linalg.det(VT.T) @np.linalg.det(U)
        C_dots_map = U@C_dots_map@VT

        r_dots_map = -C_dots_map.T@p_dots + p_map

        print("Calibration completed")

        self.tf_map_to_dots[:3,:3] = C_dots_map
        self.tf_map_to_dots[3,:3] = -r_dots_map

    def calibrate(self):
        rate = rospy.Rate(20)

        print(f"Trying to connect to drone...")
        while ((not rospy.is_shutdown() and not self.current_state.connected) or (self.current_t_map_dots is None)):
            rate.sleep()
        print(f"Connected!")

        while (not rospy.is_shutdown()):
            self.setpoint_position_pub.publish(self.current_waypoint)
            if self.pose_is_close(self.current_waypoint, self.current_map_pose):
                if len(self.waypoints_queue) == 0:
                    self.point_cloud_alignment()

                    # Land it
                    pose = np.eye(4)
                    pose[:3, 3] = [0, 0, 0.05]
                    pose = numpy_to_pose_stamped(pose, frame_id="parent_frame")
                    self.setpoint_position_pub.publish(pose)

                idx = len(self.waypoints_queue)
                map_pose_np = pose_stamped_to_numpy(self.current_map_pose)
                dots_pose_np = pose_stamped_to_numpy(self.current_dots_pose)

                # pose array 3xpoints each row (x,y,z)
                # np 4x4
                self.map_pose_array[idx, :] = map_pose_np[:3, 3]
                self.dots_pose_array[idx, :] = dots_pose_np[:3, 3]

                self.current_waypoint = self.waypoints_queue.pop()
                rate.sleep()


if __name__ == "__main__":
    rospy.init_node("px4_vicon_calibration")
    calibartion = map_to_dots()
    calibartion.calibrate()
