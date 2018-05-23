
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
import time
import sys
#import cv2
import tf
import numpy as np

class PepperNavigation:

    def __init__(self):
        rospy.Subscriber("/slam_out_pose", PoseStamped, self.update_pose)
        rospy.Subscriber("/map", OccupancyGrid, self.update_map)
        self.map = None
        self.latest_pose = None
        self.latest_map = None
        self.latest_map_pose = None
        self.listener =  tf.TransformListener()
        self.move_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    def move_to_goal(self, goal):
        self.move_publisher.publish(goal)

    def get_pose(self):
        return self.latest_pose

    def update_pose(self, data):
        self.latest_pose = data

    def update_map(self, data):
        self.map = data
	
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.resolution = data.info.resolution

        if(self.latest_pose):
            x = int(self.map_width / 2 + self.latest_pose.pose.position.x / self.resolution)
            y = int(self.map_height / 2 + self.latest_pose.pose.position.y / self.resolution)
            self.latest_map_pose = (x, y)
        data.data = [50 if x==-1 else x for x in data.data]
        self.latest_map = np.array([255 - i*2.55 for i in data.data]).reshape(data.info.width, data.info.height)
'''
if __name__ == '__main__':
    navigation = PepperNavigation()
    i = 1
    while( i < 4):
        time.sleep(1)
        i = i + 1
        if(navigation.latest_map != None and navigation.latest_map_pose != None):
            img = navigation.draw_position()
            cv2.imwrite("Map.png", img)
            cv2.waitKey(1)
'''
