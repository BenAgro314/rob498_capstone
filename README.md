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
- ssh captors@192.168.55.1

## SSH and Remote Desktop:

Put this into your `~/.ssh/config` (make that file if it doesn't exist):
```
Host jetson
    HostName 100.67.79.56 # change this to the ip of the Jetson
    User captors
    LocalForward 5900 localhost:5900
```
Then you can do `ssh jetson`, put in the password, and you should have an ssh connection.
For VNC, run `vnc` on the jetson through an ssh connection, and then on your local machine
run `vncviewer 0.0.0.0:5900`

## Debugging

- fixing permission error in `bridge_mavros`: `sudo chmod 666 /dev/ttyTHS1`
