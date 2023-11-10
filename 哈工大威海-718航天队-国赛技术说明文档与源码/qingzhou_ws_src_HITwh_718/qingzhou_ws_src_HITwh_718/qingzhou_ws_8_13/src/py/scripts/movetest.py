#! /usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('pubpose')
    turtle_vel_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    mypose=PoseStamped()
    turtle_vel_pub.publish(mypose)
    time.sleep(1)
    
    mypose=PoseStamped()
    mypose.header.frame_id='base_link'
    mypose.pose.position.x=20
    mypose.pose.position.y=0
    mypose.pose.position.z=0
    mypose.pose.orientation.x=0
    mypose.pose.orientation.y=0
    mypose.pose.orientation.z=1
    mypose.pose.orientation.w=0
    
    turtle_vel_pub.publish(mypose) 

    time.sleep(5)
