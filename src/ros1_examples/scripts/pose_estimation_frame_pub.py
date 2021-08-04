#!/usr/bin/env python

import os
import sys

import geometry_msgs.msg
import rospy
import tf
import tf2_ros


def main():
    name = os.path.split(sys.argv[0])[-1]
    rospy.init_node(name)

    # transform object to be published --------------------------------------------------------
    transform = geometry_msgs.msg.TransformStamped()
    trans = transform.transform.translation
    rot = transform.transform.rotation

    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = rospy.get_param("~parent_frame")
    transform.child_frame_id = rospy.get_param("~child_frame")

    x, y, z = [float(i) for i in os.environ["CAMERA_LINK_FRAME_XYZ"].split()]
    roll, pitch, yaw = [float(i) for i in os.environ["CAMERA_LINK_FRAME_RPY"].split()]
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    # Transform - Adjust for the different convention of the SLAMcore and ROS frames
    trans.z = -x
    trans.y = z
    trans.x = y

    quat_sc_ros = [0.5, -0.5, 0.5, 0.5]  # w-last
    quat_out = tf.transformations.quaternion_multiply(quat, quat_sc_ros)
    rot.x = quat_out[0]
    rot.y = quat_out[1]
    rot.z = quat_out[2]
    rot.w = quat_out[3]

    # broadcast and spin ----------------------------------------------------------------------
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transform)
    rospy.spin()


if __name__ == "__main__":
    main()
