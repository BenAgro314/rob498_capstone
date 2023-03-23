
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
    DIFF_DRIVE = 1

#ROT_DIST_MULT = .1  # multiplier to change effect of rotational distance in choosing correct control
#MIN_TRANS_DIST_TO_USE_ROT = 0.15  # m, robot has to be within this distance to use rot distance in cost

class LocalPlanner:
    # idea:
    # we need the drone to travel front-forwards at all times to prevent collisions
    # how: simulate a differential drive robot
    # use trajectory sampling from (v, omega), and select the best one
    # control z independently

    def __init__(self, v_max=0.5, omega_max=1.0, trans_ths=0.15, yaw_ths=0.16, mode=LocalPlannerType.DIFF_DRIVE):# , num_substeps=10, horizon=1.0):
        self.v_max = v_max
        self.omega_max = omega_max
        self.trans_ths = trans_ths
        self.mode=mode
        self.yaw_ths=yaw_ths # 10 deg
        self.a_max = 0.25
        #self.num_substeps=num_substeps
        #self.horizon=horizon
        #self.local_path_pub = rospy.Publisher('~local_path', Path, queue_size=1)
        #v_opts = np.linspace(0, self.v_max, 50)  # TODO config
        #omega_opts = np.linspace(-self.omega_max, self.omega_max, 51)
        #self.all_opts = np.array(np.meshgrid(
        #    v_opts, omega_opts)).T.reshape(-1, 2)
        pass

    def simulate_forwards(self, vel: np.array, rot_vel: np.array, start_cfg: np.array, num_substeps: Optional[int] = None, timestep: Optional[float] = None) -> np.array:
        N = vel.shape[0]
        assert vel.shape == (N,)
        assert rot_vel.shape == (N,)
        assert start_cfg.shape == (3,), "Needs to be order (x, y, theta)"
        if timestep is None:
            timestep = self.timestep
        if num_substeps is None:
            num_substeps = self.num_substeps

        vel = vel[:, None]
        rot_vel = rot_vel[:, None]

        t = np.linspace(0, timestep, num_substeps)[
            None, :]  # (1, num_substeps)
        x0 = start_cfg[0]
        y0 = start_cfg[1]
        theta0 = start_cfg[2]

        x = np.zeros((rot_vel.shape[0], t.shape[1]))  # (N, num_substeps)
        y = np.zeros((rot_vel.shape[0], t.shape[1]))

        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", category=RuntimeWarning)
            x = np.where(np.isclose(rot_vel, 0), vel * t * np.cos(theta0) + x0,
                         (vel / rot_vel) * (np.sin(rot_vel * t + theta0) - np.sin(theta0)) + x0)
            y = np.where(np.isclose(rot_vel, 0), vel * t * np.sin(theta0) + y0, -
                         (vel / rot_vel) * (np.cos(rot_vel * t + theta0) - np.cos(theta0)) + y0)
        theta = np.where(np.isclose(rot_vel, 0), theta0 *
                         np.ones_like(rot_vel * t), rot_vel * t + theta0)

        pts = np.stack((x, y), axis=-1)  # (N, self.num_substeps, 2)
        res = np.stack((x, y, theta), axis=-1)
        return res  # (N, self.num_substeps, 3)

    def get_2d_cmd(self, curr_cfg: np.array, goal_cfg: np.array):
        # 1. point towards the goal
        curr_yaw = curr_cfg[3]
        goal_vec = goal_cfg[:2] - curr_cfg[:2]
        goal_yaw = np.arctan2(goal_vec[1], goal_vec[0])
        omega = np.clip(shortest_signed_angle(curr_yaw, goal_yaw), a_min = -self.omega_max, a_max = self.omega_max)
        v = np.clip(np.linalg.norm(goal_vec), a_min = 0, a_max = self.v_max)
        if not are_angles_close(curr_yaw, goal_yaw, self.yaw_ths):
            return 0.0, omega
        else:
            return v, omega


        # 2. fly towards the goal




        #dist_from_goal = np.linalg.norm(curr_cfg[:2] - goal_cfg[:2])
        #control_horizon = np.clip(dist_from_goal / 0.5, a_min=1, a_max=5)
        #local_paths = self.simulate_forwards(
        #    self.all_opts[:, 0],
        #    self.all_opts[:, 1],
        #    curr_cfg[[0, 1, 3]],
        #    timestep=control_horizon,
        #    num_substeps=self.num_substeps,
        #)

        #cost_endpoint_trans_dist = np.linalg.norm(local_paths[:, -1, :2] - goal_cfg[:2], axis = -1)
        #dangle = local_paths[:, -1, 2] - goal_cfg[3]
        #cost_endpoint_rot_dist = np.abs(np.arctan2(np.sin(dangle), np.cos(dangle)))
        #cost_endpoint_rot_dist = np.where(cost_endpoint_rot_dist > np.pi, cost_endpoint_rot_dist - 2*np.pi,  cost_endpoint_rot_dist)
        #rot_mult = ROT_DIST_MULT if np.linalg.norm(curr_cfg[:2] - goal_cfg[:2]) < MIN_TRANS_DIST_TO_USE_ROT else 0

        #final_cost = cost_endpoint_trans_dist + rot_mult * cost_endpoint_rot_dist #+  (sim_cost / local_paths.shape[1])

        #if final_cost.size == 0: # hardcoded recovery if all options have collision
        #    control = [-.1, 0]
        #else:
        #    control = self.all_opts[final_cost.argmin()]
        #    traj = local_paths[final_cost.argmin()]
        #    self.local_path_pub.publish(se2_pose_list_to_path(traj, 'map'))
        #    self.prev_path = traj
        
        #return control

    def get_speed(self, goal_vec: np.array):
        return np.clip(np.linalg.norm(goal_vec), a_min=0, a_max=self.v_max)
        #x = np.linalg.norm(goal_vec)
        #if x >= (self.v_max**2 / (2 * self.a_max)):
        #    return self.v_max
        #else:
        #    return 2 * self.a_max * x

    def get_twist(self, t_map_d: PoseStamped, t_map_d_goal: PoseStamped) -> Twist:
        curr_cfg = get_config_from_pose_stamped(t_map_d)
        goal_cfg = get_config_from_pose_stamped(t_map_d_goal)

        goal_vec = goal_cfg[:3] - curr_cfg[:3] # (x, y, z)
        # if we are within a ths, use non holonomic control (likely safe)
        if np.linalg.norm(goal_vec) < self.trans_ths or self.mode == LocalPlannerType.NON_HOLONOMIC:
            twist_m = Twist()
            goal_vec = self.get_speed(goal_vec) * goal_vec / np.linalg.norm(goal_vec)
            twist_m.linear.x = goal_vec[0]
            twist_m.linear.y = goal_vec[1]
            twist_m.linear.z = goal_vec[2]
            twist_d = transform_twist(twist_m, np.linalg.inv(pose_stamped_to_numpy(t_map_d)))
            return twist_d
        else: # use diff drive control
            raise NotImplementedError("Only NON_HOLONOMIC controller has been implemented")
            v_x_y, omega = self.get_2d_cmd(curr_cfg, goal_cfg)
            # convert to twist in map frame
            twist = Twist()
            twist.angular.z = omega
            twist.linear.x = v_x_y * np.cos(curr_cfg[-1])
            twist.linear.y = v_x_y * np.sin(curr_cfg[-1])

            # get z cmd
            v_z = np.clip(goal_cfg[2] - curr_cfg[2], a_min=-self.v_max, a_max=self.v_max)
            twist.linear.z = v_z

            return twist
