#!/usr/bin/python3

"""
Code for sending static transforms for the robots.
"""

import rospy
import tf
from nav_msgs.msg import Odometry

def handle_robot_odom(msg, robot_name):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     msg.header.stamp,
                     robot_name + "base_footprint",
                     "map")

if __name__ == '__main__':
    rospy.init_node('robot_tf_broadcaster')
    num_of_robots = 3
    [rospy.Subscriber("/tb3_{}/odom".format(i), Odometry, handle_robot_odom, callback_args=f"/tb3_{i}/") for i in range(num_of_robots)]
    rospy.spin()
