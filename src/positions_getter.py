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
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
import cv2
import tf
import traceback
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

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
		self.connected = False
		self.map_pose = None
		rospy.init_node('talker', anonymous=True)
		self.opened_connection = True
		self.navigation = PepperNavigation()
		self.marker_array_publisher = rospy.Publisher('/people', MarkerArray, queue_size=100)
	
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

	def get_data(self):
		try:
			while self.connected:
				data = pickle.loads(self.socket.recv(BUFFER_SIZE))
				self.parse_data(data)
				i = 0
				markers = []
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
				self.marker_array_publisher.publish(markers)


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

		for person in data:
			pos_in_laser = [person[1][0], person[1][1], person[1][2]]
			br = tf.TransformBroadcaster()
			br.sendTransform(pos_in_laser,
							 tf.transformations.quaternion_from_euler(0, 0, 0),
							 rospy.Time.now(),
							 'human_in_laser',
							 'laser')
			pose = PoseStamped()
			loc = Pose()
			robot_pos = self.navigation.get_pose().pose
			loc.position.x = pos_in_laser[0]
			loc.position.y = pos_in_laser[1]
			loc.position.z = pos_in_laser[2]
			loc.orientation = robot_pos.orientation
			pose.header.frame_id = 'laser'
			pose.pose = loc

			listener = tf.TransformListener()
			try:
				listener.waitForTransform("/laser", "/map", rospy.Time(), rospy.Duration(4.0))
				pose_in_map = listener.transformPose('/odom', pose)
				pose_in_map.header.frame_id = 'odom'

				if len(person) > 2:
					person_name = person[2]
				else:
					person_name = "unknown"

				if not person_name in self.person_positions:
					self.person_positions[person_name] = [pose_in_map]
					print("Found " + person_name)
				else:
					self.person_positions[person_name][0] = pose_in_map
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



