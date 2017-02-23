#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import Image

def callback(data):
    rospy.loginfo(data)
    
    
def subscriber():
    rospy.init_node('test_sub_node', anonymous=False)

    rospy.Subscriber("/pepper_robot/camera/front/camera/image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber()
