
from enum import Enum
import rospy
from nav_msgs.msg import Path
from typing import Optional
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from offboard_py.scripts.utils import are_angles_close, pose_stamped_to_numpy, get_config_from_pose_stamped, se2_pose_list_to_path, shortest_signed_angle, transform_twist
import warnings

class LocalPlannerType(Enum):
    NON_HOLONOMIC = 0

class LocalPlanner:
    # idea:
    # we need the drone to travel front-forwards at all times to prevent collisions
    # how: simulate a differential drive robot
    # use trajectory sampling from (v, omega), and select the best one
    # control z independently

    def __init__(self, mode=LocalPlannerType.NON_HOLONOMIC):# , num_substeps=10, horizon=1.0):
        # e = [dx, dy, dz, droll, dpitch, dyaw].T (6, 1)
        # position gains
        self.kp = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.ki = np.diag([0.05, 0.05, 0.01, 0.0, 0.0, 0.0])
        self.kd = np.diag([0.3, 0.3, 0.1, 0.2, 0.2, 0.2])

        self.v_max = np.array([0.4, 0.4, 0.4, 2, 2, 2]).T

        self.prev_time = None

        self.max_integral = np.array([0.3, 0.3, 0.2, 1.0, 1.0, 1.0]).T


        self.integral = np.zeros((6, 1))
        self.previous_error = np.zeros((6, 1))


    def get_speed(self, goal_vec: np.array):
        return np.clip(np.linalg.norm(goal_vec), a_min=0, a_max=self.v_max)

    def get_twist(self, t_map_d: PoseStamped, t_map_d_goal: PoseStamped) -> Twist:
        if self.prev_time is None:
            self.prev_time = rospy.Time.now().to_sec()
            return Twist()
        dt = rospy.Time.now().to_sec() - self.prev_time

        curr_cfg = get_config_from_pose_stamped(t_map_d)
        goal_cfg = get_config_from_pose_stamped(t_map_d_goal)

        #error = (goal_cfg - curr_cfg)[:, None]
        pos_error = goal_cfg[:3] - curr_cfg[:3]
        rot_error = shortest_signed_angle(curr_cfg[3:], goal_cfg[3:])
        error = np.concatenate((pos_error, rot_error))[:, None]
        proportional = self.kp @ error 
        self.integral = self.integral + error * dt
        self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)
        integral = self.ki @ self.integral
        derivative = self.kd @ ((error - self.previous_error) / dt)

        velocity = proportional + integral + derivative
        velocity = np.clip(velocity, -self.v_max, self.v_max)


        twist_m = Twist()
        twist_m.linear.x = velocity[0,0]
        twist_m.linear.y = velocity[1,0]
        twist_m.linear.z = velocity[2,0]
        twist_m.angular.x = velocity[3,0]
        twist_m.angular.y = velocity[4,0]
        twist_m.angular.z = velocity[5,0]

        twist_d = transform_twist(twist_m, np.linalg.inv(pose_stamped_to_numpy(t_map_d)))

        self.prev_time = rospy.Time.now().to_sec()
        self.previous_error=error

        return twist_d
