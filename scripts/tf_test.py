#!/usr/bin/env python
import roslib
import sys
import rospy
import tf
from geometry_msgs.msg import Point


if __name__ == '__main__':
    
    
    rospy.init_node('tf_test', anonymous=False)
    listener = tf.TransformListener()
    tf_pub = rospy.Publisher('wrist_base_tf', Point, queue_size=5)


    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform( "/base_link", "/wrist_link",rospy.Time(0))
            result = Point()
            result.x=trans[0]
            result.y=trans[1]
            result.z=trans[2]
            tf_pub.publish(result)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print (trans)
        rate.sleep()