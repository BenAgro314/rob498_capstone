#!/usr/bin/env python3
from typing import List, Optional 

#from offboard_py.scripts.path_planner import find_traj
from offboard_py.scripts.utils import Colors
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


class RobDroneControl():

    def __init__(self):

        self.srv_launch = rospy.Service('comm/launch', Empty, self.launch_cb)
        self.srv_test = rospy.Service('comm/test', Empty, self.test_cb)
        self.srv_land = rospy.Service('comm/land', Empty, self.land_cb)
        self.srv_abort = rospy.Service('comm/abort', Empty, self.abort_cb)
        self.srv_home = rospy.Service('comm/home', Empty, self.home_cb)

        self.home_pose: Optional[PoseStamped] = None

        name = 'rob498_drone_01'  # Change 00 to your team ID
        self.sub_waypoints = rospy.Subscriber(name+'/comm/waypoints', PoseArray, self.waypoint_cb)
        self.received_waypoints: Optional[PoseArray] = None # PoseArray

        # TODO: make these arguments / config files
        self.waypoint_ths = 0.10 # used in pose_is_close
        self.on_ground_ths = 0.2
        self.launch_height = 1.6475 # check this
        self.land_height = 0.05
        self.max_speed = 0.5

        self.flight_limits = [
            [-3.0, 3.0], # x
            [-3.0, 3.0], # y
            [0.0, 3.0], # z
        ]

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        #self.setpoint_position_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.setpoint_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.pose_cb)


        #self.vicon_sub = rospy.Subscriber("/vicon/ROB498_Drone/ROB498_Drone", self.vicon_callback)
        self.local_pose_sub_sync = message_filters.Subscriber("mavros/local_position/pose", PoseStamped)
        self.vicon_pose_sub = message_filters.Subscriber("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped)

        self.time_synchronizer = message_filters.TimeSynchronizer([self.vicon_pose_sub, self.local_pose_sub_sync], queue_size=10)
        self.time_synchronizer.registerCallback(self.synchronized_vicon_callback)

        # global: vicon frame
        # map: the frame used by the px4
        # base: the frame attached to the px4
        self.t_map_global: Optional[np.array] = None
        
        # for viz / outward coms
        self.current_waypoint_pub = rospy.Publisher("rob498/current_waypoint", PoseStamped, queue_size=10)
        self.current_path_pub = rospy.Publisher("rob498/waypoint_queue", Path, queue_size=10)
        self.current_pose: Optional[PoseStamped] = None

        self.current_state = State()

        self.waypoint_queue = []
        self.current_waypoint = None 
        self.active = False

        self.waypoint_queue_num = Semaphore(0)
        self.waypoint_queue_lock = Lock()
        self.len_waypoint_queue = 0

    def synchronized_vicon_callback(self, vicon_pose: TransformStamped, mavros_pose: PoseStamped):
        t_global_dots = transform_stamped_to_numpy(vicon_pose)
        t_global_base = deepcopy(t_global_dots)
        t_global_base[2, 3] += 0.05 # check this offset distance 
        t_map_base = pose_stamped_to_numpy(mavros_pose)
        self.t_map_global = t_map_base @ np.linalg.inv(t_global_base)

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

    def pose_cb(self, pose: PoseStamped):
        self.current_pose = pose

    def state_cb(self, msg: State):
        self.current_state = msg

    def publish_current_queue(self):
        msg = Path()
        msg.header.frame_id = self.current_pose.header.frame_id
        msg.poses = [self.current_pose] + self.waypoint_queue
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
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.current_pose.header.frame_id
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
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.current_pose.header.frame_id
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
        if self.received_waypoints is None:
            print(f"Cannot test, haven't gotten waypoints ")
            return EmptyResponse()
        print(f"{Colors.GREEN}TESTING{Colors.RESET}")

        self.test_task_3()
        return EmptyResponse()

    def test_task_3(self):
        if self.t_map_global is None:
            print("Haven't recieved vicon information yet, can't do task 3")
            return 

        self.waypoint_queue_lock.acquire()
        for pose in self.received_waypoints.poses:

            # 1. convert the pose into a numpy array to transform it
            t_global_basei = pose_to_numpy(pose)
            # 2. transform it into the map frame
            t_map_basei = self.t_map_global @ t_global_basei
            # 3. turn back into a tran
            new_pose = numpy_to_pose_stamped(t_map_basei, self.current_pose.header.frame_id)
            self.waypoint_queue.append(new_pose)
            self.waypoint_queue_num.release() # semaphor.up
            self.len_waypoint_queue += 1

        self.publish_current_queue()
        self.waypoint_queue_lock.release()

    def waypoint_cb(self, msg: PoseArray):
        if self.received_waypoints is not None:
            print("Received waypoints is not None (already got them)")
            return
        if self.t_map_global is None:
            print("Haven't recieved global vicon position estimate yet")
            return
        print(f"{Colors.GREEN}RECIEVED WAYPOINTS{Colors.RESET}")
        self.received_waypoints = msg

    def can_launch(self):
        return self.current_pose.pose.position.z < self.on_ground_ths and self.len_waypoint_queue == 0

    def can_test(self):
        return self.len_waypoint_queue == 0

    def pose_is_close(self, pose1: PoseStamped, pose2: PoseStamped):
        # TOOD: include some meaasure of rotation diff
        pt1 = pose1.pose.position
        pt1 = np.array([pt1.x, pt1.y, pt1.z])
        pt2 = pose2.pose.position
        pt2 = np.array([pt2.x, pt2.y, pt2.z])
        return np.linalg.norm(pt1 - pt2) < self.waypoint_ths

    def compute_twist_command(self):
        if self.current_waypoint is None:
            return Twist()
        goal = np.array([
            [self.current_waypoint.pose.position.x],
            [self.current_waypoint.pose.position.y],
            [self.current_waypoint.pose.position.z],
        ])
        curr = np.array([
            [self.current_pose.pose.position.x],
            [self.current_pose.pose.position.y],
            [self.current_pose.pose.position.z],
        ])
        vel = np.clip(goal - curr, a_min = -self.max_speed, a_max = self.max_speed)
        res = Twist()
        res.linear.x = vel[0, 0]
        res.linear.y = vel[1, 0]
        res.linear.z = vel[2, 0]
        return res

    def run(self):
        rate = rospy.Rate(20)

        print(f"Trying to connect to drone...")
        while((not rospy.is_shutdown() and not self.current_state.connected) or (self.current_pose is None)):
            rate.sleep()
        print(f"Connected!")
        self.active = True

        while(not rospy.is_shutdown()):
            #self.setpoint_position_pub.publish(self.current_waypoint)
            self.setpoint_vel_pub.publish(self.compute_twist_command())
            if self.current_waypoint is None or self.pose_is_close(self.current_waypoint, self.current_pose):
                #self.queue_lock.acquire()
                if self.len_waypoint_queue > 0:
                    self.current_waypoint = self.waypoint_queue_pop()
                    self.current_waypoint_pub.publish(self.current_waypoint)
                #self.queue_lock.release()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("rob498_drone")
    drone_control = RobDroneControl()
    drone_control.run()
