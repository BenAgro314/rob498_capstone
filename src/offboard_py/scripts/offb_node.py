#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_srvs.srv import Empty, EmptyResponse


class DroneController():

    def __init__(self):
        self.current_state = State()

        rospy.init_node('offb_node_py') 

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.srv_launch = rospy.Service('comm/launch', Empty, self.callback_launch)
        self.rate = rospy.Rate(20)
        print("Trying to Connect")
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()
        print("Connected!")
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        self.run()
    
    def callback_launch(self, request):
        self.handle_launch()
        return EmptyResponse()

    def handle_launch(self):
        print("Launching!")

    def run(self):
        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()


        while(not rospy.is_shutdown()):
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

    def state_cb(self, msg):
        self.current_state = msg

if __name__ == "__main___":
    pass


