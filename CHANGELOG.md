# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

## Version `v0.2.0` (published on 2021/09/17)

### Added

### Changed

- Use local point cloud, published by the SLAMcore ROS Wrapper
  (`sensor_msgs::PointCloud2`) instead of `depthimage_to_laserscan` and the
  pseudo-laser to do obstacle avoidance

  - The point cloud is published at `/slamcore/local_point_cloud`

- Publish the full model of the robot platform in use. This includes the kobuki
  robot, a model of the camera and the assembly of mounting plates the camera
  stands on.
- Change the semantics of `CAMERA_LINK_FRAME_RPY` and `CAMERA_LINK_FRAME_XYZ`
  variables. They now encode the transformation of the `base_footprint` frame
  relative to the `slamcore/base_link` frame.

### Deprecated

### Removed

### Fixed

- The transform of `base_footprint` relative to `slamcore/base_link` was
  hardcoded in the `kobuki_live_navigation.launch` file. It is now set via the
  `CAMERA_LINK_FRAME_*` environment variables

### Security

## Version `v0.1.0` (published on 2021/07/07)

Initial release of ROS1 Navigation Demo
