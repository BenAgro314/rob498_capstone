#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_srvs.srv import Empty, EmptyResponse


current_state = State()
pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

# Callback handlers
def handle_launch():
    global pose
    print('Launch Requested. Your drone should take off.')
    pose.pose.position.z = 1.5

def handle_land():
    global pose
    print('Land Requested. Your drone should land.')
    pose.pose.position.z = 0.2

# Service callbacks
def callback_launch(request):
    handle_launch()
    return EmptyResponse()

def callback_land(request):
    handle_land()
    return EmptyResponse()

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    #rospy.wait_for_service("/mavros/cmd/arming")
    #arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    srv_launch = rospy.Service('comm/launch', Empty, callback_launch)
    srv_launch = rospy.Service('comm/land', Empty, callback_land)
    #rospy.wait_for_service("/mavros/set_mode")
    #set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()


    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    #offb_set_mode = SetModeRequest()
    #offb_set_mode.custom_mode = 'OFFBOARD'

    #arm_cmd = CommandBoolRequest()
    #arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        # if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
        #     if(set_mode_client.call(offb_set_mode).mode_sent == True):
        #         rospy.loginfo("OFFBOARD enabled")
        #     
        #     last_req = rospy.Time.now()
        # else:
        #     if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
        #         if(arming_client.call(arm_cmd).success == True):
        #             rospy.loginfo("Vehicle armed")
        #     
        #         last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()
