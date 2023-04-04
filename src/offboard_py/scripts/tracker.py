#!/usr/bin/env python3

from offboard_py.scripts.utils import get_config_from_transformation, pointcloud2_to_numpy, quaternion_to_euler, transform_to_numpy
import rospy
import cv2
import tf2_ros
import matplotlib.pyplot as plt
import tf.transformations
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from skimage.draw import disk, polygon

class Tracker:

    def __init__(self):
        self.points = None
        self.cyl_sub= rospy.Subscriber("det_points", PointCloud2, callback = self.cyl_callback)

        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.occ_map_pub = rospy.Publisher('occ_map', OccupancyGrid, queue_size=10)

        self.map_bounds = [-5, -5, 5, 5] # min_x, min_y, max_x, max_y
        self.map_res = 0.1

        self.map_shape = (
                int((self.map_bounds[3] - self.map_bounds[1]) // self.map_res),
                int((self.map_bounds[2] - self.map_bounds[0]) // self.map_res)
            )
        self.logits = np.zeros(
            self.map_shape
        )
        self.radius=0.3
        self.alpha = 1.0
        self.beta = -0.05
        self.fov = (-np.pi/8, np.pi/8)
        self.range = 10
        self.wall_width = 0.1

    def publish_occupancy_grid(self):
        map = np.exp(self.logits) / (1 + np.exp(self.logits))

        height, width = map.shape
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.header.frame_id = 'map'

        occupancy_grid.info = MapMetaData()
        occupancy_grid.info.resolution = self.map_res
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        occupancy_grid.info.origin.position.x = self.map_bounds[0]
        occupancy_grid.info.origin.position.y = self.map_bounds[1]
        occupancy_grid.info.origin.position.z = 0
        occupancy_grid.info.origin.orientation.x = 0
        occupancy_grid.info.origin.orientation.y = 0
        occupancy_grid.info.origin.orientation.z = 0
        occupancy_grid.info.origin.orientation.w = 1

        occupancy_grid.data = (map.flatten() * 100).astype(np.int8).tolist()

        self.occ_map_pub.publish(occupancy_grid)

    def cyl_callback(self, msg):
        #pos = msg.pose.position

        image_time = msg.header.stamp

        pos_mask = np.zeros_like(self.logits, dtype = bool)
        neg_mask = np.zeros_like(self.logits, dtype = bool)

        #pt_imx = np.array([pos.x, pos.y, pos.z, 1])[:, None] # (3, 1)
        self.tf_buffer.can_transform('map', 'base_link', image_time, timeout=rospy.Duration(5))
        t_map_base = self.tf_buffer.lookup_transform(
        "map", "base_link", image_time).transform
        t_base_imx = self.tf_buffer.lookup_transform(
        "base_link", "imx219", image_time).transform
        x_base, y_base, _, _, _, yaw_base = get_config_from_transformation(t_map_base)
        t_map_base  = transform_to_numpy(t_map_base)
        t_base_imx  = transform_to_numpy(t_base_imx)

        cam_angle = yaw_base - np.pi / 2
        min_fov_pt = x_base + self.range * np.cos(cam_angle + self.fov[0]), y_base + self.range * np.sin(cam_angle + self.fov[0]), 0
        max_fov_pt = x_base + self.range * np.cos(cam_angle + self.fov[1]), y_base + self.range * np.sin(cam_angle + self.fov[1]), 0
        base_ind = self.point_to_ind(np.array([x_base, y_base, 0])[:, None])
        min_fov_ind = self.point_to_ind(np.array(min_fov_pt)[:, None])
        max_fov_ind = self.point_to_ind(np.array(max_fov_pt)[:, None])

        imx_points = pointcloud2_to_numpy(msg)
        if len(imx_points) == 0:
            poly_rr, poly_cc = polygon([base_ind[0], min_fov_ind[0], max_fov_ind[0]], [base_ind[1], min_fov_ind[1], max_fov_ind[1]], shape = self.logits.shape)
            neg_mask[poly_rr, poly_cc] = True
        else:
            for pt_imx in imx_points:
                pt_imx = np.concatenate((pt_imx[:, None], np.array([[1]])), axis = 0) # (3, 1)
                pt_imx_left = pt_imx.copy()
                pt_imx_right = pt_imx.copy()

                pt_imx_left[0] -= self.radius
                pt_imx_right[0] += self.radius

                t_map_imx = t_map_base @ t_base_imx

                pt_map_left = t_map_imx @ pt_imx_left
                pt_map_right = t_map_imx @ pt_imx_right

                pt_map_left[2, 0] = 0.0
                pt_map_right[2, 0] = 0.0

                left_ind = self.point_to_ind(pt_map_left)                 
                right_ind = self.point_to_ind(pt_map_right)                 

                angle_left = np.arctan2(pt_map_left[1] - y_base, pt_map_left[0] - x_base)
                angle_right = np.arctan2(pt_map_right[1] - y_base, pt_map_right[0] - x_base)

                left_fov_pt = (x_base + self.range * np.cos(angle_left))[0], (y_base + self.range * np.sin(angle_left))[0], 0
                right_fov_pt = (x_base + self.range * np.cos(angle_right))[0], (y_base + self.range * np.sin(angle_right))[0], 0

                left_fov_ind = self.point_to_ind(np.array(left_fov_pt)[:, None])
                right_fov_ind = self.point_to_ind(np.array(right_fov_pt)[:, None])
                
                poly_rr, poly_cc = polygon([base_ind[0], min_fov_ind[0], left_fov_ind[0], left_ind[0], right_ind[0], right_fov_ind[0], max_fov_ind[0]], [base_ind[1], min_fov_ind[1],  left_fov_ind[1], left_ind[1], right_ind[1], right_fov_ind[1], max_fov_ind[1]], shape = self.logits.shape)
                neg_mask[poly_rr, poly_cc] = True

                pt_map = t_map_imx @ pt_imx
                pt_map[2, 0] = 0.0 # zero out z

                rr, cc = disk(self.point_to_ind(pt_map), self.radius // self.map_res)
                rr[rr >= pos_mask.shape[0]]  = pos_mask.shape[0] - 1
                rr[rr < 0]  = 0
                cc[cc >= pos_mask.shape[1]]  = pos_mask.shape[1] - 1
                cc[cc < 0]  = 0
                pos_mask[rr, cc] = True

        neg_mask = np.logical_and(neg_mask, ~pos_mask)

        self.logits[neg_mask] += self.beta
        self.logits[pos_mask] += self.alpha
        self.logits = np.clip(self.logits, a_min = -2, a_max = 10)

        width_inds = int(round(self.wall_width/self.map_res))
        self.logits[-width_inds:] = 10
        self.logits[:, -width_inds:] = 10
        self.logits[:width_inds] = 10
        self.logits[:, :width_inds] = 10

        self.publish_occupancy_grid()

    def point_to_ind(self, pt):
        # pt.shape == (3, 1) or (2, 1)
        x = pt[0, 0]
        y = pt[1, 0]

        row_ind = (y - self.map_bounds[1]) // (self.map_res) 
        col_ind = (x - self.map_bounds[0]) // (self.map_res) 

        return (row_ind, col_ind)



if __name__ == "__main__":
    rospy.init_node("tracker")
    detector = Tracker()
    rospy.spin()
