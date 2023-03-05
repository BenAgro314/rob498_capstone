# Optical Flow calculation


Within this directory, simply call make flow to generate the output binary flow_run.

NOTE: On different machines, the location of the opencv installation may be different. Changing the PKG_CONFIG_PATH and possibly LD_FLAGS may be necessary to connect properly to wherever opencv is installed on the jetson.

TO-DO:
- Ensure cv can access our HD Camera on the Jetson
- Create a ROS node in c++ inside the main file, and publish the flow readings to mavros_msgs/OpticalFlowRad Message.
- Test thoroughly... I suspect doing something wrong here might have the potential for state estimations to go out of control and make the drone go haywire.
