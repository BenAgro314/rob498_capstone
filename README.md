# rob498_capstone

## Links:

- [ROB498-flight Repo](https://github.com/utiasSTARS/ROB498-flight)
- [ROB498 Docker Repo](https://github.com/manx52/ROB498)
- [ROB498 Docker image](https://hub.docker.com/r/utrarobosoccer/rob498)
- [Tutorial For Setting up PX4 <-> Nano](https://www.youtube.com/watch?v=Brkk0ZmnGgs)
- [PX4 VIO docs](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry.html)
- [Wifi Debugging](https://forums.developer.nvidia.com/t/jetson-nano-wifi/72269/21)
	- Maybe this command worked: sudo iw dev wlan0 set power_save off
- [Challenge Task Page](https://q.utoronto.ca/courses/299314/pages/challenge-tasks-midterm-video-and-final-report?wrap=1)
- [Amazing docker files for jetson](https://github.com/dusty-nv/jetson-containers)
- [another docker thing to look at](https://github.com/timongentzsch/Jetson_Ubuntu20_Images)

## SSH Using Jetson's MicroUSB Port
- ssh jetson@192.168.55.1

To get the ip address for U of T wifi, ssh via Microusb, type "ifconfig" and use the ip address under wlan0 -> inet

## SSH and Remote Desktop:

Put this into your `~/.ssh/config` (make that file if it doesn't exist):
```
Host jetson
    HostName 100.67.79.56 # change this to the ip of the Jetson on U of T wifi
    User jetson
    LocalForward 5900 localhost:5900
```
Then you can do `ssh jetson`, put in the password, and you should have an ssh connection.
For VNC, run `vnc` on the jetson through an ssh connection, and then on your local machine
run `vncviewer 0.0.0.0:5900`

## Debugging

- fixing permission error in `bridge_mavros`: `sudo chmod 666 /dev/ttyTHS1`


## Setting up the Jetson

1. Follow instructions here to flash the Jetson with Ubuntu 20.04: https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image
2. Install ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu
3. `sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`
4. `rosdep init`
5. `rosdep update`
6. `sudo apt install python3-catkin-tools`
7. Clone our repo `git clone https://github.com/BenAgro314/rob498_capstone`, and rename it to `~/catkin_ws`
8. `cd ~/catkin_ws/` and `catkin build` 
9. `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc` and `source ~/.bashrc`
10. `sudo apt install ros-noetic-mavros ros-noetic-mavros-extras`
11. `cd ~` and `wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh`
12. `chmod +x install_geographiclib_datasets.sh`
13. `sudo ./install_geographiclib_datasets.sh`
14. `sudo apt install libpcl1 ros-noetic-octomap-*`
15. `sudo apt-get install ros-noetic-realsense2-camera`
16. Follow the instructions in the top answer here `https://stackoverflow.com/questions/62134563/how-to-give-permission-to-intel-realsense-camera-on-ubuntu`
17. Follow the instructions here to install the Wifi drivers: `https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/hardware/jetson_nano.md` 

