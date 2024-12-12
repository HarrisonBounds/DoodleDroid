# DoodleDroid
<img src="IMG_0814.HEIC" alt="Robot Drawing In Progress" width="300">

https://github.com/user-attachments/assets/d116fecf-2de2-481f-8cd0-af1a74ca4b40

## Secondary roles:
Dev-ops:  David
Reliability: Harrison
Code hygienist: Christian
Integration: Han
Hardware Engineer: Yanni

# Overview
A franka robot arm is used to portraits as line art. Users can take a photo of themselves or others which the robot will convert to pen strokes and draw them on a paper detected and localized using april tags.

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
- Print out the 11x17" page with the 4 april tags on it. Do not scale it.
- Place the page on the Franka table.
- Follow instructions on https://nu-msr.github.io/ros_notes/ros2/franka.html to connect to the Franka arm and start the controllers and moveit.
- On your machine, run `ros2 launch doodle_droid all.launch.xml` to launch the system.
- Start by calibrating the system run `ros2 service call /calibrate  std_srvs/srv/Empty`. Once calibration is complete, you can:
- Take a photo: run `ros2 service call /take_photo  std_srvs/srv/Empty`. Retake your photo if desired, or if the output looks good,
- Draw the photo: run `ros2 service call /draw  std_srvs/srv/Empty`. Wait patiently for your commissioned piece.
- Once the robot returns to the `ready' pose. Disable with the enabling device and collect the masterpiece. Prep new image.

