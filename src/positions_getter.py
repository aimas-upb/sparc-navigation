import sys
task_path = "../../hall-moving-scenario/"
sys.path.insert(0, task_path)

import socket
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
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
import cv2
import tf
import traceback
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from TaskManagement.tasksManagement import Task

#keystrokes
import select


IP = "192.168.0.158"
PORT = 3000
BUFFER_SIZE = 10000

class TcpClient:
	def __init__(self):
		self.ip = IP
		self.port = PORT
		self.person_positions = {}
		self.location_positions = {}
		self.connected = False
		self.map_pose = None
		rospy.init_node('talker', anonymous=True)
		self.opened_connection = True
		self.navigation = PepperNavigation()
		self.people_marke_array_publisher = rospy.Publisher('/people', MarkerArray, queue_size=100)
		self.locations_marker_array_publisher = rospy.Publisher('/qrCodes', MarkerArray, queue_size=100)

	def connect(self):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		try:
			self.socket.connect((self.ip, self.port))
			print("Connected to server.")
			self.connected = True
		except:
			print("Connection failed.")
			sys.exit(0)

	def move_to_person(self, person_name):
		if person_name in self.person_positions:
			self.navigation.move_to_goal(client.person_positions[key][0])
			return True
		return False

	def move_to_map_pose(self):
		if self.map_pose:
			print("moving")
			self.navigation.move_to_goal(self.map_pose)
		else:
			return False

	def publish_persons(self):
		markers = []
		i = 0
		for person_name in self.person_positions:
			from copy import deepcopy
			marker = Marker(
				type=Marker.TEXT_VIEW_FACING,
				id=i,
				lifetime=rospy.Duration(2.0),
				pose=deepcopy(self.person_positions[person_name][0].pose),
				scale=Vector3(0.2, 0.2, 0.2),
				header=Header(frame_id='odom'),
				color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
				text=person_name)

			marker.pose.position.z += 0.3
			markers.append(marker)
			i += 1
			marker2 = Marker(
				type=Marker.SPHERE,
				id=i,
				lifetime=rospy.Duration(2.0),
				pose=self.person_positions[person_name][0].pose,
				scale=Vector3(0.2, 0.2, 0.2),
				header=Header(frame_id='odom'),
				color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
				text=person_name)
			markers.append(marker2)
			i += 1
		self.people_marke_array_publisher.publish(markers)

	def publish_locations(self):
		markers = []
		i = 0
		for location in self.location_positions:
			from copy import deepcopy
			marker = Marker(
				type=Marker.TEXT_VIEW_FACING,
				id=i,
				lifetime=rospy.Duration(2.0),
				pose=deepcopy(self.location_positions[location][0].pose),
				scale=Vector3(0.2, 0.2, 0.2),
				header=Header(frame_id='odom'),
				color=ColorRGBA(0.0, 0.0, 1.0, 0.8),
				text=location)

			marker.pose.position.z += 0.3
			markers.append(marker)
			i += 1
			marker2 = Marker(
				type=Marker.SPHERE,
				id=i,
				lifetime=rospy.Duration(2.0),
				pose=self.location_positions[location][0].pose,
				scale=Vector3(0.2, 0.2, 0.2),
				header=Header(frame_id='odom'),
				color=ColorRGBA(0.0, 0.0, 1.0, 0.8),
				text=location)
			markers.append(marker2)
			i += 1
		self.locations_marker_array_publisher.publish(markers)

	def get_data(self):
		try:
			while self.connected:
				raw_data = self.socket.recv(BUFFER_SIZE)
				try:
					data = pickle.loads(raw_data)
				except:
					continue
				self.parse_data(data)
				self.publish_persons()
				self.publish_locations()

		except KeyboardInterrupt:
			print("Script interrupted by user, shutting down.")

		except Exception as e:
			print("Exception: " + str(e))
			traceback.print_exc()

		finally:
			self.socket.close()


	def parse_data(self, data):
		if not data or len(data) == 0:
			return

		for task in data:
			if task.type == 'show_on_map':
				pose = PoseStamped()
				loc = Pose()
				robot_pos = self.navigation.get_pose().pose
				loc.position.x = task.coordinates[0]
				loc.position.y = task.coordinates[1]
				loc.position.z = task.coordinates[2]
				loc.orientation = robot_pos.orientation
				pose.header.frame_id = 'laser'
				pose.pose = loc

				listener = tf.TransformListener()
				try:
					listener.waitForTransform("/laser", "/map", rospy.Time(), rospy.Duration(4.0))
					pose_in_map = listener.transformPose('/odom', pose)
					pose_in_map.header.frame_id = 'odom'

					if task.person_name != '':
						person_name = task.person_name
						if not person_name in self.person_positions:
							self.person_positions[person_name] = [pose_in_map]
							print("Found " + person_name)
						else:
							self.person_positions[person_name][0] = pose_in_map

					if task.location != '':
						location = task.location
						if not location in self.location_positions:
							self.location_positions[location] = [pose_in_map]
							print("Found " + location)
						else:
							self.location_positions[location][0] = pose_in_map
				except:
					pass

if __name__ == "__main__":
	from threading import Thread
	client = TcpClient()
	client.connect()
	thread = Thread(target=client.get_data)
	thread.start()
	key = ''
	print("Press q to exit / person name to move")
	while(key != 'q'):
		key = sys.stdin.readline().strip()
		if client.move_to_person(key):
			print("Moving to " + key)
		else:
			print("Cannot move to " + key)

	client.connected = False
	thread.join()



