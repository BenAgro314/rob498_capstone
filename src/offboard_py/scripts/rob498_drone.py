#!/usr/bin/env python3

import rospy
from copy import deepcopy
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_srvs.srv import Empty, EmptyResponse

class RobDroneControl():

    def __init__(self):

        self.srv_launch = rospy.Service('comm/launch', Empty, self.launch_cb)
        self.srv_test = rospy.Service('comm/test', Empty, self.test_cb)
        self.srv_land = rospy.Service('comm/land', Empty, self.land_cb)
        self.srv_abort = rospy.Service('comm/abort', Empty, self.abort_cb)

        # TODO: make these arguments / config files
        self.waypoint_ths = 0.2 # used in pose_is_close
        self.on_ground_ths = 0.2
        self.launch_height = 1.5 # check this
        self.num_launch_waypoints = 3
        self.num_land_waypoints = 3
        self.land_height = 0.15

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        self.setpoint_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.pose_cb)
        self.current_pose = None

        self.current_state = State()

        self.waypoint_queue = []
        self.current_waypoint = PoseStamped()
        self.active = False

    def pose_cb(self, pose: PoseStamped):
        self.current_pose = pose

    def state_cb(self, msg: State):
        self.current_state = msg

    def launch_cb(self, request: Empty):
        if not self.active:
            print("Cannot launch, not connected yet")
            return EmptyResponse()
        if not self.can_launch():
            print(f"Cannot launch, not on ground, or waypoint queue is not empty: {self.waypoint_queue}")
            return EmptyResponse()

        print("Launching")
        for i in range(self.num_launch_waypoints):
            pose = deepcopy(self.current_pose)
            pose.pose.position.z = self.launch_height * (float(i+1) / float(self.num_launch_waypoints))
            self.waypoint_queue.append(pose)
        return EmptyResponse()

    def land_cb(self, request: Empty):
        if not self.active:
            print("Cannot land, not connected yet")
            return EmptyResponse()
        if len(self.waypoint_queue) != 0:
            print("Cannot land, there are waypoints on the queue")
            return EmptyResponse()

        print("Landing")
        current_height = self.current_pose.pose.position.z
        for i in range(self.num_land_waypoints):
            pose = deepcopy(self.current_pose)
            pose.pose.position.z = current_height * (1.0 - (float(i+1) / float(self.num_land_waypoints))) + self.land_height
            self.waypoint_queue.append(pose)
        return EmptyResponse()

    def abort_cb(self, request: Empty):
        self.current_waypoint = self.current_pose
        print("Aborting!")
        self.waypoint_queue = []
        return EmptyResponse()

    def test_cb(self, request: Empty):
        return EmptyResponse()

    def can_launch(self):
        return self.current_pose.pose.position.z < self.on_ground_ths and len(self.waypoint_queue) == 0

    def pose_is_close(self, pose1: PoseStamped, pose2: PoseStamped):
        # TOOD: include some meaasure of rotation diff
        pt1 = pose1.pose.position
        pt1 = np.array([pt1.x, pt1.y, pt1.z])
        pt2 = pose2.pose.position
        pt2 = np.array([pt2.x, pt2.y, pt2.z])
        return np.linalg.norm(pt1 - pt2) < self.waypoint_ths

    def run(self):
        rate = rospy.Rate(20)

        print(f"Trying to connect to drone...")
        while((not rospy.is_shutdown() and not self.current_state.connected) or (self.current_pose is None)):
            rate.sleep()
        print(f"Connected!")
        self.active = True

        while(not rospy.is_shutdown()):
            self.setpoint_pub.publish(self.current_waypoint)
            if self.pose_is_close(self.current_waypoint, self.current_pose):
                if len(self.waypoint_queue) > 0:
                    self.current_waypoint = self.waypoint_queue.pop(0)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("rob498_drone")
    drone_control = RobDroneControl()
    drone_control.run()
