#!/usr/bin/env python
from re import T
import rospy
import time
import tf
if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')
    s=0
    print(1)
    # Pos=False
    # while not Pos:
    #     rospy.get_param("Pos",Pos)
    #     continue
    listener = tf.TransformListener()
    
    
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        x,y=trans[0],trans[1]
        rospy.set_param("RP1",0)
        rospy.set_param("RP2",0)
        rospy.set_param("RP3",0)
        xpoint1=rospy.get_param("px1")
        ypoint1=rospy.get_param("py1")
        xpoint2=rospy.get_param("px2")
        ypoint2=rospy.get_param("py2")
        xpoint3=rospy.get_param("px3")
        ypoint3=rospy.get_param("py3")
        rospy.set_param("pos_x",x)
        rospy.set_param("pos_y",y)
        if (x-xpoint1)**2+(y-ypoint1)**2<=0.4 and -0.1 <= (rot[2]+rot[3]) <= 0.1:
            rospy.set_param("RP1",1)
        if (x-xpoint2)**2<=0.09 and (y-ypoint2)**2<=0.04:
            rospy.set_param("RP2",1)
        if (x-xpoint3)**2+(y-ypoint3)**2<=0.15:
            rospy.set_param("RP3",1)
        else:
            pass
        time.sleep(0.1)
