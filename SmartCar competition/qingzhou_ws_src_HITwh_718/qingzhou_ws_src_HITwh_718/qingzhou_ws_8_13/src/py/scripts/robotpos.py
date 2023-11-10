#!/usr/bin/env python
import rospy
import time
import tf
if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0)) 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print('position:',trans)
        print('orient:',rot)
        time.sleep(0.5)
