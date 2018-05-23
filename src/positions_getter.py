import socket
import sys
import pickle
import numpy as np
import collections

from PepperLocalization import PepperNavigation
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from std_msgs.msg import String
import cv2
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

#keystrokes
import select



IP = "192.168.0.158"
PORT = 3000
BUFFER_SIZE = 10000

class TcpClient:
	def __init__(self):
		self.ip = IP
		self.port = PORT
		self.path = []
		self.connected = False
		self.path_pub = rospy.Publisher("/human_path", Path, queue_size=100)
		self.pose_pub = rospy.Publisher("/human_pose", PoseStamped, queue_size=100)
		self.map_pose = None
		rospy.init_node('talker', anonymous=True)
		self.opened_connection = True
		self.navigation = PepperNavigation()
	
	def connect(self):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		try:
			self.socket.connect((self.ip, self.port))
			print("Connected to server.")
			self.connected = True
		except:
			print("Connection failed.")
			sys.exit(0)

	def move_to_map_pose(self):
		if self.map_pose:
			print("moving")
			self.navigation.move_to_goal(self.map_pose)
		else:
			return False

	def get_data(self):
		try:
			while self.connected:
				data = pickle.loads(self.socket.recv(BUFFER_SIZE))
				self.parse_data(data)
				if self.map_pose:
					self.pose_pub.publish(self.map_pose)

		except KeyboardInterrupt:
			print("Script interrupted by user, shutting down.")

		except Exception as e:
			print("Exception: " + str(e))

		finally:
			self.socket.close()

	def publish_path(self):
		pose_list = list()
		#path to send the Rviz
		my_path = Path()
		#my_path.header.stamp = rospy.Time.now()
		my_path.header.frame_id = 'laser'

		br = tf.TransformBroadcaster()
		print(self.path[-1])
		br.sendTransform(self.path[-1],
						 tf.transformations.quaternion_from_euler(0, 0, 0),
						 rospy.Time.now(),
						 'human_in_laser',
						 'laser')

		pose = PoseStamped()
		loc = Pose()
		robot_pos = self.navigation.get_pose().pose
		loc.position.x = self.path[-1][0]
		loc.position.y = self.path[-1][1]
		loc.position.z = self.path[-1][2]
		loc.orientation = robot_pos.orientation
		pose.header.frame_id = 'laser'
		pose.pose = loc

		listener = tf.TransformListener()
		listener.waitForTransform("/laser", "/map", rospy.Time(), rospy.Duration(4.0))
		self.map_pose = listener.transformPose('/odom', pose)
		self.map_pose.header.frame_id = 'odom'

		#make the poses into posestamped
		'''
		for position in self.path:
			loc = Pose()
			pose = PoseStamped()
			robot_pos = self.navigation.get_pose().pose
			loc.position.x = position[0] + robot_pos.position.x
			loc.position.y = position[1] + robot_pos.position.y
			loc.position.z = position[2] + robot_pos.position.z
			loc.orientation = robot_pos.orientation
			pose.header.frame_id = 'laser'
			pose.pose = loc

			# pose.header.frame_id = '/odom'
			# pose.header.stamp = rospy.Time.now()

			my_path.poses.append(pose)


		self.path_pub.publish(my_path)
		'''

	def parse_data(self, data):
		if not data or len(data) == 0:
			return

		for person in data:
			print("Person: " + str(person[0]))
			print("\tx: " + str(person[1][0]))
			print("\ty: " + str(person[1][1]))
			print("\tz: " + str(person[1][2]))

			robotPos = self.navigation.get_pose().pose.position
			pos_in_map = [person[1][0], person[1][1] , person[1][2]]
			self.path.append(pos_in_map)
			if len(self.path) > 10:
				self.path.pop(0)

			if len(self.path) > 2:
				self.publish_path()

if __name__ == "__main__":
	from threading import Thread
	client = TcpClient()
	client.connect()
	thread = Thread(target=client.get_data)
	thread.start()
	key = ''
	print("Press q to exit / w to move")
	while(key != 'q'):
		key = sys.stdin.read(1)
		if(key == 'w'):
			client.move_to_map_pose()
	client.connected = False
	thread.join()



