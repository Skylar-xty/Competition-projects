#! /usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
#def __init__(self, name, data_class, callback=None, callback_args=None,queue_size=None, buff_size=DEFAULT_BUFF_SIZE, tcp_nodelay=False):
def callback(data):
    print(2)

if __name__ == '__main__':
    rospy.init_node('test')
    print(1)
    sub = rospy.Subscriber('move_base/result',MoveBaseActionResult,callback)
    #print(sub)
