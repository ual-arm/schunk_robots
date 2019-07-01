#!/usr/bin/env python

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('summit_tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            #print (trans[0], trans[1])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

            continue

        if (2.4 >= trans[0] >= 2.1) and (1.3 >= trans[1] >= 1.0):
            print ('Summit in Zone 1: Closer to the edge of the table')

        if (1.6 >= trans[0] >= 1.1) and (1.8 >= trans[1] >= 1.4):
            print('Summit in Zone 2: Between ABB and Powerball')

        if (0.35 >= trans[0] >= 0.0) and (0.0 >= trans[1] >= -0.07):
            print('Summit out of work zones')

        rate.sleep()
