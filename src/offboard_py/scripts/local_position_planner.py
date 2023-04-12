#!/usr/bin/env python3
from enum import IntEnum
from typing import List
import cv2
from std_srvs.srv import Empty, EmptyResponse
from threading import Lock
from copy import deepcopy
import time
from functools import partial
from offboard_py.scripts.utils import are_angles_close, config_to_pose_stamped, get_config_from_pose_stamped, numpy_to_pose, shortest_signed_angle, slerp_pose
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
import networkx as nx
from nav_msgs.msg import Path
from std_msgs.msg import Header
import pyastar2d

USE_COLOR = True

def coll_free(p1, p2, coll_fn, steps=10):
    pts = np.linspace(p1, p2, steps)
    return not np.any(coll_fn(pts))

def blur_image_direction(image, direction, px):
    # Define the kernel size and shape based on the direction
    # Perform the convolution using the kernel
    if direction == 'POS_X':
        kernel = np.ones((1, px), np.uint8)
        blurred_image = cv2.dilate(image, kernel, iterations = 1, anchor = (px-1,0))
    elif direction == 'POS_Y':
        kernel = np.ones((px, 1), np.uint8)
        blurred_image = cv2.dilate(image, kernel, iterations = 1, anchor = (0, px-1))
    elif direction == 'NEG_X':
        kernel = np.ones((1, px), np.uint8)
        blurred_image = cv2.dilate(image, kernel, iterations =1, anchor =(0,0))
    elif direction == 'NEG_Y':
        kernel = np.ones((px, 1), np.uint8)
        blurred_image = cv2.dilate(image, kernel, iterations = 1, anchor =(0,0))
    else:
        raise ValueError("Invalid direction. Must be POS_X, POS_Y, NEG_X, or NEG_Y.")

    return blurred_image

class LocalPlanner:

    def __init__(self):


        self.green_map_lock = Lock()
        self.green_map = None

        self.red_map_lock = Lock()
        self.red_map = None

        self.map_width = None
        self.map_height = None
        self.map_res = None
        self.map_origin = None

        self.waypoint_trans_ths = 0.15 # 0.08 # used in pose_is_close
        self.waypoint_yaw_ths = np.deg2rad(10.0) # used in pose_is_close

        self.path_pub = rospy.Publisher('local_plan', Path, queue_size=10)

        self.vehicle_radius = 0.45
        self.blur_dist = 2.0
        self.current_path = None


        self.green_map_sub= rospy.Subscriber("green_occ_map", OccupancyGrid, callback = self.green_map_callback)
        self.red_map_sub= rospy.Subscriber("red_occ_map", OccupancyGrid, callback = self.red_map_callback)

    

    def point_to_ind(self, pt):
        # pt.shape == (3, 1) or (2, 1)
        assert self.map_res is not None
        x = pt[0, 0]
        y = pt[1, 0]

        row_ind = (y - self.map_origin.y) // (self.map_res) 
        col_ind = (x - self.map_origin.x) // (self.map_res) 

        return (row_ind, col_ind)

    def path_to_path_message(self, path, t_map_d, t_map_d_goal, frame_id='map'):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = frame_id
        n = len(path)
        for i, point1 in enumerate(path):
            x1 = point1[1] * self.map_res + self.map_origin.x
            y1 = point1[0] * self.map_res + self.map_origin.y
            p = slerp_pose(t_map_d.pose, t_map_d_goal.pose, rospy.Time(0), rospy.Time(1), rospy.Time(i/(n-1)), 'map')
            z, yaw = get_config_from_pose_stamped(p)[[2, 5]]
            pose = config_to_pose_stamped(x1, y1, z, yaw, frame_id=frame_id)
            path_msg.poses.append(pose)
        return path_msg

    def red_map_callback(self, map_msg):
        with self.red_map_lock:
            self.map_width = map_msg.info.width
            self.map_height = map_msg.info.height
            self.map_res = map_msg.info.resolution
            self.map_origin = map_msg.info.origin.position
            self.red_map = np.array(map_msg.data, dtype=np.uint8).reshape((self.map_height, self.map_width, 1))

    def green_map_callback(self, map_msg):
        with self.green_map_lock:
            self.map_width = map_msg.info.width
            self.map_height = map_msg.info.height
            self.map_res = map_msg.info.resolution
            self.map_origin = map_msg.info.origin.position
            self.green_map = np.array(map_msg.data, dtype=np.uint8).reshape((self.map_height, self.map_width, 1))


    def plan(self, weights, t_map_d, t_map_d_goal, collision_fn):
        x1, y1 = get_config_from_pose_stamped(t_map_d)[:2]
        x2, y2 = get_config_from_pose_stamped(t_map_d_goal)[:2]

        #weights = np.ones_like(occ_map).astype(np.float32)
        #weights[occ_map > 50] = np.inf
        r1, c1 = self.point_to_ind(np.array([x1, y1, 0])[:, None]) # turn those into indices into occ map
        r2, c2 = self.point_to_ind(np.array([x2, y2, 0])[:, None])
        success = True
        path = pyastar2d.astar_path(weights, (int(r1), int(c1)), (int(r2), int(c2)), allow_diagonal=True)
        if path is None:
            success = False
            print(f"Failed to find path! Staying still instead")
            path = [(r1, c1)]
        elif len(path) == 1:
            path = [(r2, c2)] * 2
        # shorten path
        shorter_path = []
        i = 0
        while i < len(path):
            shorter_path.append(path[i])
            j = i+1
            while j < len(path)-1 and coll_free(path[i], path[j], collision_fn):
                j+=1
            i = j
        path_msg = self.path_to_path_message(shorter_path, t_map_d, t_map_d_goal, 'map')
        # to correct for discretization
        path_msg.poses[0] = t_map_d
        if success:
            path_msg.poses[-1] = t_map_d_goal
        path_msg.poses = self.smooth_path(path_msg.poses, t_map_d, t_map_d_goal)
        return path_msg


    def pose_is_close(self, pose1: PoseStamped, pose2: PoseStamped):
        # TOOD: include some meaasure of rotation diff
        cfg1 = get_config_from_pose_stamped(pose1)
        cfg2 = get_config_from_pose_stamped(pose2)
        trans_is_close =  np.linalg.norm(cfg1[:3] - cfg2[:3]) < self.waypoint_trans_ths

        yaw_is_close = are_angles_close(cfg1[-1], cfg2[-1], self.waypoint_yaw_ths)
        return trans_is_close and yaw_is_close

    def path_has_collision(self, path: Path, collision_fcn):
        for pose in path.poses:
            x1,y1 = get_config_from_pose_stamped(pose)[:2] # get current position (x,y)
            r1, c1 = self.point_to_ind(np.array([x1, y1, 0])[:, None]) # turn those into indices into occ map
            if collision_fcn(np.array([r1, c1])):
                return True
        return False
        
    def smooth_path(self, poses: List[PoseStamped], t_map_d: PoseStamped, t_map_d_goal: PoseStamped) -> List[PoseStamped]:
        smoothed_poses = [poses[0]]
        for p1, p2 in zip(poses[:-1], poses[1:]):
            x1, y1, z1, _, _, yaw1 = get_config_from_pose_stamped(p1)
            x2, y2, z2, _, _, yaw2 = get_config_from_pose_stamped(p2)
            dyaw = np.abs(shortest_signed_angle(yaw1, yaw2))
            dd = 0 #np.sqrt((x1 - x2)**2  + (y1 - y2)**2) # + (z1 - z2)**2)
            n = max(1, int((dd / 0.5)) + int((dyaw / np.deg2rad(5))))
            for i in range(n):
                p = slerp_pose(p1.pose, p2.pose, rospy.Time(0), rospy.Time(1), rospy.Time((i+1)/n), 'map')
                p.header.stamp = rospy.Time.now()
                smoothed_poses.append(p)
        n = len(smoothed_poses)
        if n == 1:
            smoothed_poses[0] = t_map_d
            return smoothed_poses
        for i in range(len(smoothed_poses)):
            p = slerp_pose(t_map_d.pose, t_map_d_goal.pose, rospy.Time(0), rospy.Time(1), rospy.Time(i/(n-1)), 'map')
            p.header.stamp = rospy.Time.now()
            _, _, z, _, _, yaw = get_config_from_pose_stamped(p)
            sp = smoothed_poses[i]
            x, y = get_config_from_pose_stamped(sp)[:2]
            smoothed_poses[i] = config_to_pose_stamped(x, y, z, yaw, 'map')
        return smoothed_poses 

    def run(self, t_map_d: PoseStamped, t_map_d_goal: PoseStamped) -> Path:

        t = time.time()

        with self.green_map_lock:
            if self.green_map is None:
                return
            green_occ_map = self.green_map.copy()[:, :, 0]
        with self.red_map_lock:
            if self.red_map is None:
                return
            red_occ_map = self.red_map.copy()[:, :, 0]


        red_occ_map = (red_occ_map > 50).astype(np.uint8)
        green_occ_map = (green_occ_map > 50).astype(np.uint8)

        if USE_COLOR:
            x1, y1 = get_config_from_pose_stamped(t_map_d)[:2]
            x2, y2 = get_config_from_pose_stamped(t_map_d_goal)[:2]
            r1, c1 = self.point_to_ind(np.array([x1, y1, 0])[:, None]) # turn those into indices into occ map
            r2, c2 = self.point_to_ind(np.array([x2, y2, 0])[:, None])

            dr = r2 - r1
            dc = c2 - c1
            #print('dr', dr)
            #print('dc', dc)
            #print()
            blur_inds = int(round(self.blur_dist / self.map_res))
            if np.abs(dr) > np.abs(dc):
                if dr > 0:
                    #dir = Dir.POS_X
                    red_occ_blur = blur_image_direction(red_occ_map, "POS_X", blur_inds)
                    green_occ_blur = blur_image_direction(green_occ_map, "NEG_X", blur_inds)
                else:
                    red_occ_blur = blur_image_direction(red_occ_map, "NEG_X", blur_inds)
                    green_occ_blur = blur_image_direction(green_occ_map, "POS_X", blur_inds)
            else:
                if dc > 0:
                    red_occ_blur = blur_image_direction(red_occ_map, "NEG_Y", blur_inds)
                    green_occ_blur = blur_image_direction(green_occ_map, "POS_Y", blur_inds)
                else:
                    red_occ_blur = blur_image_direction(red_occ_map, "POS_Y", blur_inds)
                    green_occ_blur = blur_image_direction(green_occ_map, "NEG_Y", blur_inds)



        # Define structuring element
        occ_mask = np.logical_or(red_occ_map > 0 , green_occ_map > 0).astype(np.uint8)
        wall_width = 0.1
        wall_inds = max(int(round(wall_width/self.map_res)), 1)
        occ_mask[-wall_inds:] = 1
        occ_mask[:, -wall_inds:] = 1
        occ_mask[:wall_inds] = 1
        occ_mask[:, :wall_inds] = 1
        buff_inds = 2 * int(round(self.vehicle_radius / self.map_res)) + 1
        kernel = np.ones((buff_inds, buff_inds), np.uint8) # add on 3 * map_res of 
        # Apply dilation filter
        dilated_map = cv2.dilate(occ_mask, kernel, iterations=1)        
        #dilated_map = np.logical_or(np.logical_or(dilated_map, green_occ_blur), red_occ_blur)

        

        weights = np.ones_like(occ_mask).astype(np.float32)
        weights[occ_mask] = np.inf
        weights[np.logical_and(~occ_mask, dilated_map)] = 100

        if USE_COLOR:
            weights[np.logical_and(np.logical_and(~occ_mask, ~dilated_map), red_occ_blur)] = 2 * np.pi / self.map_res
            weights[np.logical_and(np.logical_and(~occ_mask, ~dilated_map), green_occ_blur)] = 2 * np.pi / self.map_res

        
        
        def collision_fn_strict(pt):
            if len(pt.shape) == 2:
                pts = np.round(pt).astype(np.int32)
            else:
                row=int(round(pt[0]))
                col=int(round(pt[1]))
                pts = np.array([row, col])[None, :] # (1, 2)
            #circle_pts = self.inds_to_robot_circle(pts)
            rows = pts[..., 0]
            cols = pts[..., 1]
            return np.any(weights[rows, cols] > 1, axis = 0)
        
        def collision_fn(pt):
            if len(pt.shape) == 2:
                pts = np.round(pt).astype(np.int32)
            else:
                row=int(round(pt[0]))
                col=int(round(pt[1]))
                pts = np.array([row, col])[None, :] # (1, 2)
            #circle_pts = self.inds_to_robot_circle(pts)
            rows = pts[..., 0]
            cols = pts[..., 1]
            return np.any(occ_mask[rows, cols] == 1, axis = 0)

        if self.current_path is None or len(self.current_path.poses) == 0 or not self.pose_is_close(t_map_d_goal, self.current_path.poses[-1]) or self.path_has_collision(self.current_path, collision_fn):
            self.current_path = self.plan(weights, t_map_d, t_map_d_goal, collision_fn_strict)
        else:
            # check if we are near first waypoint on path
            assert len(self.current_path.poses) > 0
            if self.pose_is_close(t_map_d, self.current_path.poses[0]):
                self.current_path.poses = self.current_path.poses[1:]
            if len(self.current_path.poses) == 0:
                self.current_path.poses = [t_map_d_goal]

        #self.path = path_msg

        self.path_pub.publish(self.current_path)

        return self.current_path
