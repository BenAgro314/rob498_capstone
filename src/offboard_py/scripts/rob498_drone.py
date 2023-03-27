#!/usr/bin/env python3
from typing import List, Optional
from offboard_py.scripts.local_planner import LocalPlanner, LocalPlannerType 

#from offboard_py.scripts.path_planner import find_traj
from offboard_py.scripts.utils import Colors, are_angles_close, config_to_transformation_matrix, get_config_from_pose_stamped, make_sphere_marker, numpy_to_transform_stamped, pose_stamped_to_transform_stamped, shortest_signed_angle, slerp_pose, transform_stamped_to_pose_stamped, transform_twist
from offboard_py.scripts.utils import pose_to_numpy, transform_stamped_to_numpy, pose_stamped_to_numpy, numpy_to_pose_stamped
from threading import Semaphore, Lock
import rospy
from copy import deepcopy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, PoseArray, TransformStamped
from mavros_msgs.msg import State
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import Path
import message_filters
import tf2_ros
from visualization_msgs.msg import Marker

USE_SLERP=False
SWEEP_HGT=False
USE_ORIENTATION=False
PERP=False

class RobDroneControl():

    def __init__(self):

        
        name = 'rob498_drone_01'  # Change 00 to your team ID
        self.srv_launch = rospy.Service(name + '/comm/launch', Empty, self.launch_cb)
        self.srv_test = rospy.Service(name + '/comm/test', Empty, self.test_cb)
        self.srv_land = rospy.Service(name + '/comm/land', Empty, self.land_cb)
        self.srv_abort = rospy.Service(name + '/comm/abort', Empty, self.abort_cb)
        self.srv_home = rospy.Service(name + '/comm/home', Empty, self.home_cb)

        #self.broadcaster = tf2_ros.TransformBroadcaster()

        self.home_pose: Optional[PoseStamped] = None

        self.sub_waypoints = rospy.Subscriber(name+'/comm/waypoints', PoseArray, self.waypoint_cb)
        self.received_waypoints: Optional[PoseArray] = None # PoseArray

        # TODO: make these arguments / config files
        self.waypoint_trans_ths = 0.08 # used in pose_is_close
        self.waypoint_yaw_ths = np.deg2rad(10.0) # used in pose_is_close
        self.on_ground_ths = 0.2
        self.launch_height = 1.6475 # check this
        self.land_height = 0.05
        self.max_speed = 0.5 # m/s
        self.task_ball_radius = 0.15

        self.local_planner = LocalPlanner() 

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        #self.setpoint_position_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.setpoint_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.pose_cb)


        self.vicon_sub = rospy.Subscriber("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, self.vicon_callback)
        self.test_ready = False

        # global: vicon frame
        # map: the frame used by the px4
        # base: the frame attached to the px4
        self.t_map_global: np.array = np.eye(4) # assume no transformation at first
        
        # for viz / outward coms
        self.current_path_pub = rospy.Publisher("rob498/waypoint_queue", Path, queue_size=10)
        self.current_t_map_dots: Optional[PoseStamped] = None
        self.prev_vicon_pose: Optional[PoseStamped] = None
        self.prev_t_map_dots: Optional[PoseStamped] = None

        self.current_state = State()

        self.waypoint_queue = []
        self.current_waypoint = None 
        self.active = False

        self.waypoint_queue_num = Semaphore(0)
        self.waypoint_queue_lock = Lock()
        self.current_pose_lock = Lock()
        self.len_waypoint_queue = 0

        self.marker_pub = rospy.Publisher('sphere_marker', Marker, queue_size=10)
        self.t_dots_base = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0.05],
                [0, 0, 0, 1],
            ]
        )
        #self.broadcaster.sendTransform(numpy_to_transform_stamped(self.t_dots_base, frame_id='dots_link', child_frame_id='base_link'))
        self.t_base_dots = np.linalg.inv(self.t_dots_base)

    def publish_sphere_marker(self, x: float, y: float, z:float, r: float):
        marker = make_sphere_marker(x, y, z, r)
        self.marker_pub.publish(marker)

    def vicon_callback(self, vicon_pose: TransformStamped):
        if USE_SLERP:
            vicon_pose.header.stamp=rospy.Time.now()
            vicon_pose=transform_stamped_to_pose_stamped(vicon_pose)
            if self.prev_vicon_pose is None or self.current_t_map_dots is None or self.prev_t_map_dots is None:
                self.prev_vicon_pose = vicon_pose
                return
            # this filters out if we are sent two consecutive vicon messages at the same time
            # seems to occur during sim, maybe not during real
            if vicon_pose.header.stamp == self.prev_vicon_pose.header.stamp:
                print(f"{Colors.CYAN}Warning: received two consective vicon poses with the same timestamp {Colors.RESET}")
                self.prev_vicon_pose = vicon_pose
                return
            interp_t_global_dots = False
            # do as little as possible in the lock
            with self.current_pose_lock:
                if vicon_pose.header.stamp > self.current_t_map_dots.header.stamp:
                    interp_time = self.current_t_map_dots.header.stamp
                    pose1 = self.prev_vicon_pose
                    pose2 = vicon_pose
                    interp_t_global_dots = True
                else:
                    interp_time = vicon_pose.header.stamp
                    pose1 = deepcopy(self.prev_t_map_dots)
                    pose2 = deepcopy(self.current_t_map_dots)
                    interp_t_global_dots = False
            # slerp
            interp_pose = pose_stamped_to_numpy(
                    slerp_pose(
                    pose1.pose,
                    pose2.pose,
                    pose1.header.stamp,
                    pose2.header.stamp,
                    interp_time,
                    frame_id=""
                )
            )
            if interp_t_global_dots:
                t_global_dots = interp_pose
                t_map_dots = pose_stamped_to_numpy(self.current_t_map_dots)
            else:
                t_global_dots = pose_stamped_to_numpy(vicon_pose)
                t_map_dots = interp_pose
            self.t_map_global = t_map_dots @ np.linalg.inv(t_global_dots)
            self.prev_vicon_pose = vicon_pose
        else:
            if self.current_t_map_dots is not None:
                with self.current_pose_lock:
                    t_global_dots = transform_stamped_to_numpy(vicon_pose)
                    self.t_map_global = pose_stamped_to_numpy(self.current_t_map_dots) @ np.linalg.inv(t_global_dots)
        #self.broadcaster.sendTransform(numpy_to_transform_stamped(self.t_map_global, frame_id='map', child_frame_id='global'))

    def waypoint_queue_push(self, pose: PoseStamped):
        self.waypoint_queue_lock.acquire()

        self.waypoint_queue.append(pose)

        self.len_waypoint_queue += 1
        self.waypoint_queue_lock.release()
        self.waypoint_queue_num.release()

    def waypoint_queue_pop(self):
        self.waypoint_queue_num.acquire() # semaphor.down
        self.waypoint_queue_lock.acquire()

        res = self.waypoint_queue.pop(0)

        self.len_waypoint_queue -= 1
        self.waypoint_queue_lock.release()
        return res

    def pose_cb(self, t_map_base: PoseStamped):
        #self.broadcaster.sendTransform(pose_stamped_to_transform_stamped(t_map_base, parent_frame_id='map', child_frame_id='base_link'))
        t_map_base = pose_stamped_to_numpy(t_map_base)
        t_map_dots = numpy_to_pose_stamped(t_map_base @ self.t_base_dots, frame_id='map')
        with self.current_pose_lock:
            self.prev_t_map_dots = self.current_t_map_dots
            self.current_t_map_dots = t_map_dots

    def state_cb(self, msg: State):
        self.current_state = msg

    def publish_current_queue(self):
        msg = Path()
        msg.header.frame_id = self.current_t_map_dots.header.frame_id
        msg.poses = [self.current_t_map_dots] + self.waypoint_queue
        self.current_path_pub.publish(msg)

    def home_cb(self, request: Empty):
        if self.home_pose is None:
            print("Cannot go home, home is not defined")
            return EmptyResponse()
        self.waypoint_queue_lock.acquire()
        # empty waypoint queue
        while self.len_waypoint_queue > 0:
            self.waypoint_queue.pop(0)
            self.waypoint_queue_num.acquire()
            self.len_waypoint_queue -= 1
        # add home waypoint
        self.home_pose.header.stamp = rospy.Time.now()
        self.waypoint_queue.append(self.home_pose)
        self.waypoint_queue_num.release()
        self.len_waypoint_queue += 1
        # clear received waypoints for re-test
        self.received_waypoints = None
        self.publish_current_queue()
        self.waypoint_queue_lock.release()
        return EmptyResponse()

    def launch_cb(self, request: Empty):
        #self.queue_lock.acquire()

        if not self.active:
            print("Cannot launch, not connected yet")
            return EmptyResponse()
        if not self.can_launch():
            print(f"Cannot launch, not on ground, or waypoint queue is not empty: {self.len_waypoint_queue}")
            return EmptyResponse()

        print(f"{Colors.GREEN}LAUNCHING{Colors.RESET}")
        pose = deepcopy(self.current_t_map_dots)
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.z = self.launch_height
        self.home_pose = pose
        self.waypoint_queue_push(pose)

        self.publish_current_queue()

        #self.queue_lock.release()
        return EmptyResponse()

    def land_cb(self, request: Empty):
        #self.queue_lock.acquire()
        if not self.active:
            print("Cannot land, not connected yet")
            return EmptyResponse()
        if self.len_waypoint_queue != 0: # TODO: dangerous
            print("Cannot land, there are waypoints on the queue")
            return EmptyResponse()

        print(f"{Colors.GREEN}LANDING{Colors.RESET}")
        pose = deepcopy(self.current_t_map_dots)
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.z = self.land_height
        self.waypoint_queue_push(pose)

        self.publish_current_queue()
        #self.queue_lock.release()
        return EmptyResponse()

    def abort_cb(self, request: Empty):
        print(f"{Colors.RED}ABORTING{Colors.RESET}")
        exit()
        return EmptyResponse()

    def test_cb(self, request: Empty):
        #self.queue_lock.acquire()
        if not self.active:
            print("Cannot test, not connected yet")
            return EmptyResponse()
        if not self.can_test():
            print(f"Cannot test, not launched or waypoint queue is not empty : {self.len_waypoint_queue}")
            return EmptyResponse()
        self.test_ready=True

        return EmptyResponse()

    def test_task_3(self):
        if self.t_map_global is None:
            print("Haven't recieved vicon information yet, can't do task 3")
            return 

        self.waypoint_queue_lock.acquire()

        list_t_map_dots = [pose_stamped_to_numpy(self.current_t_map_dots)]
        for pose in self.received_waypoints.poses:
            t_global_dotsi = pose_to_numpy(pose)
            t_map_dotsi = self.t_map_global @ t_global_dotsi
            list_t_map_dots.append(t_map_dotsi)

        if USE_ORIENTATION:
            prev_yaw = None
            for t_map_dotsi, t_map_dotsip1 in zip(list_t_map_dots[:-1], list_t_map_dots[1:]):
                pose = numpy_to_pose_stamped(t_map_dotsi, self.current_t_map_dots.header.frame_id)
                next_pose = numpy_to_pose_stamped(t_map_dotsip1, self.current_t_map_dots.header.frame_id)

                x1, y1, z1 = get_config_from_pose_stamped(pose)[:3]
                x2, y2, z2 = get_config_from_pose_stamped(next_pose)[:3]
                #t_global_dotsi = pose_stamped_to_numpy(pose)
                #t_global_dotsip1 = pose_stamped_to_numpy(next_pose)
                yaw = np.arctan2(y2 - y1, x2 - x1)
                if PERP:
                    yaw += np.pi/2
                
                if prev_yaw is not None:
                    pose = numpy_to_pose_stamped(config_to_transformation_matrix(x1, y1, z1, prev_yaw))
                self.waypoint_queue.append(pose)
                self.waypoint_queue_num.release() # semaphor.up
                self.len_waypoint_queue += 1

                pose_facing_next = numpy_to_pose_stamped(config_to_transformation_matrix(x1, y1, z2, yaw))
                self.waypoint_queue.append(pose_facing_next)
                self.waypoint_queue_num.release() # semaphor.up
                self.len_waypoint_queue += 1
                prev_yaw = yaw
        else:
            for t_map_dotsi in list_t_map_dots:
                new_pose = numpy_to_pose_stamped(t_map_dotsi, self.current_t_map_dots.header.frame_id)
                if SWEEP_HGT:
                    new_pose_bottom = deepcopy(new_pose)
                    new_pose_bottom.pose.position.z -= self.task_ball_radius / 2
                    self.waypoint_queue.append(new_pose_bottom)
                    self.waypoint_queue_num.release() # semaphor.up
                    self.len_waypoint_queue += 1

                    new_pose_top = deepcopy(new_pose)
                    new_pose_top.pose.position.z += self.task_ball_radius / 2
                    self.waypoint_queue.append(new_pose_top)
                    #self.waypoint_queue.append(new_pose)
                    self.waypoint_queue_num.release() # semaphor.up
                    self.len_waypoint_queue += 1
                else:
                    self.waypoint_queue.append(new_pose)
                    self.waypoint_queue_num.release() # semaphor.up
                    self.len_waypoint_queue += 1

        self.publish_current_queue()
        self.waypoint_queue_lock.release()

    def waypoint_cb(self, msg: PoseArray):
        if not self.test_ready:
            print("Didn't call test yet")
            return
        if self.received_waypoints is not None:
            return
        print(f"{Colors.GREEN}RECIEVED WAYPOINTS{Colors.RESET}")
        self.received_waypoints = msg
        self.test_task_3()

    def can_launch(self):
        return self.current_t_map_dots.pose.position.z < self.on_ground_ths and self.len_waypoint_queue == 0

    def can_test(self):
        return self.len_waypoint_queue == 0

    def pose_is_close(self, pose1: PoseStamped, pose2: PoseStamped):
        # TOOD: include some meaasure of rotation diff
        cfg1 = get_config_from_pose_stamped(pose1)
        cfg2 = get_config_from_pose_stamped(pose2)
        trans_is_close =  np.linalg.norm(cfg1[:3] - cfg2[:3]) < self.waypoint_trans_ths
        if USE_ORIENTATION:
            yaw_is_close = are_angles_close(cfg1[-1], cfg2[-1], self.waypoint_yaw_ths)
            return trans_is_close and yaw_is_close
        else:
            return trans_is_close

    def compute_twist_command(self):
        if self.current_waypoint is None:
            return Twist()
        twist_dots = self.local_planner.get_twist(self.current_t_map_dots, self.current_waypoint)
        twist_base = transform_twist(twist_dots, self.t_base_dots)
        if not USE_ORIENTATION: # no angular velocity cmd
            twist_base.angular.x = 0.0
            twist_base.angular.y = 0.0
            twist_base.angular.z = 0.0
        return twist_base

    def run(self):
        rate = rospy.Rate(20)

        print(f"Trying to connect to drone...")
        while((not rospy.is_shutdown() and not self.current_state.connected) or (self.current_t_map_dots is None)):
            rate.sleep()
        print(f"Connected!")
        self.active = True

        while(not rospy.is_shutdown()):
            #self.setpoint_position_pub.publish(self.current_waypoint)
            self.setpoint_vel_pub.publish(self.compute_twist_command())
            if self.current_waypoint is None or self.pose_is_close(self.current_waypoint, self.current_t_map_dots):
                #self.queue_lock.acquire()
                if self.len_waypoint_queue > 0:
                    self.current_waypoint = self.waypoint_queue_pop()
                    cfg = get_config_from_pose_stamped(self.current_waypoint)
                    self.publish_sphere_marker(cfg[0], cfg[1], cfg[2], self.waypoint_trans_ths)
                #self.queue_lock.release()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("rob498_drone")
    drone_control = RobDroneControl()
    drone_control.run()
