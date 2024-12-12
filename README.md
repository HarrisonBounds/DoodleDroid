# DoodleDroid

https://docs.google.com/document/d/1tCsjUFVBGqud-HylWmlEa1OkyHOy-n-hFuSvpV8IUJU/edit?usp=sharing

## Secondary roles:
Dev-ops:  David
Reliability: Harrison
Code hygienist: Christian
Integration: Han
Hardware Engineer: Yanni

# Quickstart
## Install
Follow NU-MSR github to install realsense
https://nu-msr.github.io/hackathon/computer_setup.html#org20c2832

Follow instructions on https://nu-msr.github.io/ros_notes/ros2/franka.html to setup Franka environment

Also install:
```
sudo apt install ros-jazzy-usb-cam
sudo apt install ros-jazzy-apriltag-ros
sudo apt install ros-jazzy-image-pipeline
```

## Build
- colcon build
- source install/setup.bash
## Run
- Follow instructions on https://nu-msr.github.io/ros_notes/ros2/franka.html to connect to the Franka arm and start the controllers and moveit.
- On your machine, run `ros2 launch doodle_droid all.launch.xml` to launch the system.
- Start by calibrating the system run `ros2 service call /calibrate  std_srvs/srv/Empty`. Once calibration is complete, you can:
- Take a photo: run `ros2 service call /take_photo  std_srvs/srv/Empty`. Retake your photo if desired, or if the output looks good,
- Draw the photo: run `ros2 service call /draw  std_srvs/srv/Empty`

# Demos:
- Photos [**ELEPHANT FILL ME IN**]
- Videos [**ELEPHANT FILL ME IN**]

