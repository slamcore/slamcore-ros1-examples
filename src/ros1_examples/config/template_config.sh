# This is a template file for live navigation runs with either the kobuki or the
# turtlebot3. You can copy it to create your own version and edit it according
# to your setup. See the corresponding keys for more information
unset CAMERA_LINK_FRAME_XYZ
unset CAMERA_LINK_FRAME_RPY
unset SESSION_FILE
unset CONFIG_FILE
unset ODOM_READING_TOPIC

# Transformation of your camera with respect to the base of the robot
# Use the ros1_examples/view_model.launch file to find compute and/or verify an
# estimate of this transform.
# Following is a rough transformation of the Realsense camera when you're using
# the in-house mounting plates.
export CAMERA_LINK_FRAME_XYZ="0.09338891 -0.00284919 0.0254389"
# Edit this accordingly after calibration
export CAMERA_LINK_FRAME_RPY="0 0 0"

# export SESSION_FILE=TODO_PATH_TO_SESSION_FILE
# export CONFIG_FILE=TODO_ADD_PATH_TO_CONFIG_FILE
export ODOM_READING_TOPIC="/odom"
