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

## For remote desktop:

Note: this ip address seems to change:
- Connect to Jetson via microusb 
- ssh rob498@192.168.55.1
- Enter password

- Run `vnc` on the jetson
- Run `vncviewer 0.0.0.0:5900` on laptop
