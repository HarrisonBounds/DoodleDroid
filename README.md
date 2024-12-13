# DoodleDroid

# Overview
A franka robot arm is used to draw portraits as line art. Users can take a photo of themselves or others which the robot will convert to pen strokes and draw them on a paper detected and localized using april tags.



<p align="center">
  <img src="doodle_droid.png" alt="Robot Drawing In Progress" width="300">
</p>

https://github.com/user-attachments/assets/d116fecf-2de2-481f-8cd0-af1a74ca4b40

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
- Print out the 11x17" page with the 4 april tags on it. Do not scale it.
- Place the page on the Franka table.
- Follow instructions on https://nu-msr.github.io/ros_notes/ros2/franka.html to connect to the Franka arm and start the controllers and moveit.
- On your machine, run `ros2 launch doodle_droid all.launch.xml` to launch the system.
- Start by calibrating the system run `ros2 service call /calibrate  std_srvs/srv/Empty`. Once calibration is complete, you can:
- Take a photo: run `ros2 service call /take_picture  std_srvs/srv/Empty`. Retake your photo if desired, or if the output looks good,
- Draw the photo: run `ros2 service call /draw  std_srvs/srv/Empty`. Wait patiently for your commissioned piece.
- Once the robot returns to the `ready' pose. Disable with the enabling device and collect the masterpiece. Prep new image.


# Nodes, Launchfiles, and System Architecture
## Nodes
- calibrator: places robot in calibration pose and looks for april tags on the paper worspace. Publishes Pose of paper origin (xyz & quaternion) on the `surface_pose` topic.
- image processor: when `/take_picture` service is called, captures image from USB camera or file (depending on configuration) and processes it into line art. publishes lines over `/new_image` topic which the route planner listens to.
- route planner: listens on the `/new_image` topic. when sets of polylines are received, the route planner node uses an approximation of the [Travelling Salseman Problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem) to order the polylines in preparation for efficient drawing. These line segments are chained together with pen-up and pen-down motions to ensure no lines are drawn to connect sequential drawn lines. The route planner applies the calibration received from the calibration node to transform the drawing route into the paper frame for robot execution. When the `/draw` service is called, this node uses moveit to draw the image. 
## Launchfiles:
- all.launch.xml
    -  starts all nodes described above, as well as 3rd party nodes (e.g. intel realsense node,  and usb cam node) for the entire system to function as described above.
