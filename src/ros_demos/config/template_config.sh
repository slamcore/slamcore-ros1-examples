# This is a template file for live navigation runs with either the kobuki or the
# turtlebot3. You can copy it to create your own version and edit it according
# to your setup. See the corresponding keys for more information
unset CAMERA_LINK_FRAME_XYZ
unset CAMERA_LINK_FRAME_RPY
unset SESSION_FILE
unset MAP_FOR_NAVIGATION

# XYZ Transformation of your Realsense with respect to the base of the robot
# Use the ros_demos/view_model.launch file to find compute and/or verify an
# estimate of this transform
# Following is a rough transformation of the Realsense camera when you're using
# the in-house mounting plates.
export CAMERA_LINK_FRAME_XYZ="0.120 0.0 0.240"
# Edit this accordingly after calibration
export CAMERA_LINK_FRAME_RPY="0.0 0.0 0.0"

# Set one of these 2 - mutually exclusively.
#
# If you set the session file then the map will be extracted from that session
# file and it will be used for navigation as well. If you want to run SLAM
# only (i.e., no multisession, thus no loaded session), then you have to manually
# specify the map filepath via the MAP_FOR_NAVIGATION environment variable
# export SESSION_FILE="<path/to/the/session/you/want/to/use>"
export MAP_FOR_NAVIGATION="<path/to/the/map/you/want/to/use/for/nav>"
