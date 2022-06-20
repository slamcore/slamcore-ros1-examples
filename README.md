# SLAMcore ROS Melodic Demos and Tutorials

This repository contains packages to go alongside our dedicated demonstration
tutorials which will allow you to get SLAMcore up and running with ROS Melodic
in no time. The current example demonstrates how to integrate
[SLAMcore](https://www.slamcore.com/) with the [ROS Navigation
Stack](http://wiki.ros.org/move_base) and was designed for a
[Kobuki](http://kobuki.yujinrobot.com/about2/) mobile base, with a [Jetson
Xavier
NX](https://www.nvidia.com/en-gb/autonomous-machines/embedded-systems/jetson-xavier-nx/)
and [RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/) depth
camera. Nevertheless, by following our [Navigation Stack Integration
Tutorial](https://docs.slamcore.com/navstack-integration.html) you will be able
to modify the launch and config files in this repo to suit your setup.

## Demo Video

[![](http://img.youtube.com/vi/TMkKkJk6538/maxresdefault.jpg)](https://www.youtube.com/watch?v=TMkKkJk6538)

## About SLAMcore

[SLAMcore](https://www.slamcore.com/) offers commercial-grade visual-inertial
simultaneous localisation and mapping (SLAM) software for real-time autonomous
navigation on robots and drones.

## Getting started

### Requirements

To run the demo in this repository, you will need:

- Ubuntu 18.04
- [SLAMcore ROS Wrapper](https://docs.slamcore.com/ros1-wrapper.html)
- [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
- Intel RealSense D435i or D455 Depth Camera (D455 on Jetson platforms
  requires a DKMS patch. See our
  [slamcore-dkms](https://github.com/slamcore/slamcore-dkms) repository for details.)

For more details on the full requirements to run SLAMcore, visit our [Requirements Page](https://docs.slamcore.com/requirements.html).

### Installation

- First, install ROS Melodic and the SLAMcore ROS Wrapper by following the
  [SLAMcore ROS1 Wrapper Tutorial](https://docs.slamcore.com/ros1-wrapper.html)
- Next, to learn how to integrate SLAMcore's positioning and mapping ROS wrapper
  with the [ROS Navigation Stack](http://wiki.ros.org/move_base) follow the
  [Navigation Stack Integration
  Tutorial](https://docs.slamcore.com/navstack-integration.html). 
- The tutorial will take you through all the steps from cloning this repository
  and setting up dependencies to creating a map, navigating and sending goals
  from a remote machine.
