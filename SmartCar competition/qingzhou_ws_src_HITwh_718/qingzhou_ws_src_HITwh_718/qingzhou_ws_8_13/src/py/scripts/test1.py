import rospy
import time
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

if __name__ == '__main__':
    rospy.init_node('test')
    sub = rospy.Subscriber('move_base/result',MoveBaseActionResult,callback)
    print(sub)
