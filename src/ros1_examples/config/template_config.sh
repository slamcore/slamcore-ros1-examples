# This is a template file for live navigation runs with either the kobuki or the
# turtlebot3. You can copy it to create your own version and edit it according
# to your setup. See the corresponding keys for more information
unset CAMERA_LINK_FRAME_XYZ
unset CAMERA_LINK_FRAME_RPY
unset SESSION_FILE
unset CONFIG_FILE
unset ODOM_READING_TOPIC

# Transformation: Robot base_footprint frame with regards to the SLAMcore frame
# of reference. Currently the SLAMcore frame has the following configuration
# (assumming the camera is mounted looking forwards and parallel to the robot
# base):
#
# Z forwards
# X left
# Y down
#
# Use the ros1_examples/view_model.launch file to compute and/or verify an
# estimate of this transform.
# Following is a rough transformation of the Realsense camera when you're using
# the in-house mounting plates and 3D printed camera mount.
# Edit this accordingly after calibration
export CAMERA_LINK_FRAME_XYZ="-0.0028 0.2310 -0.0934"
export CAMERA_LINK_FRAME_RPY="0.000 -1.571 1.571"

# export SESSION_FILE=TODO_PATH_TO_SESSION_FILE
# export CONFIG_FILE=TODO_ADD_PATH_TO_CONFIG_FILE
export ODOM_READING_TOPIC="/odom"
