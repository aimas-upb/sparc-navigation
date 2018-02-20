import sys
naoqi_path = "/home/sparc-308/workspace/stefania/pynaoqi-python2.7/lib/python2.7/site-packages"
sys.path.insert(0, naoqi_path)

import qi
import cv2
import math
from copy import deepcopy

ip = "192.168.0.115"
port = 9559

ENTER_KEY_CODE = 13
ESC_KEY_CODE = 27


class Navigate(object):
    def __init__(self, app):
        print("Initializing navigation...")

        super(Navigate, self).__init__()
        app.start()
        session = app.session

        self.navigation = session.service("ALNavigation")
        self.motion = session.service("ALMotion")
        self.motion.moveInit()

        # Disables autonomous life and makes the robot stand up right.
        self.alife = session.service("ALAutonomousLife")
        if self.alife.getState() != "disabled":
            self.alife.setState("disabled")

        # Set initial position.
        self.motion.moveInit()
        self.motion.wakeUp()

        self.robot_position = None
        self.initial_robot_map_position = None
        self.initial_robot_real_position = None
        self.trajectory_points = []

        self.map_angle = -0.23404
        self.pixel_distance = 0.05
        self.image = cv2.imread("map.png", 0)
        if not self.image.data:
            print("Map image could not be opened.")
            sys.exit(1)

        cv2.namedWindow("Map", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Map", self.on_mouse_click, 0)

        print("Initialization completed.")


    def on_mouse_click(self, event, x, y, c, w):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.trajectory_points.append((x, y))
            if self.initial_robot_map_position == None:
                self.initial_robot_map_position = (x, y)

        elif event == cv2.EVENT_MBUTTONDOWN:
            self.robot_position = (x, y)

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.trajectory_points.pop()
            if len(self.trajectory_points) == 0:
                self.initial_robot_map_position == None


    def get_robot_position(self):
        real_position = self.motion.getRobotPosition(True)
        if self.initial_robot_real_position == None:
            self.initial_robot_real_position = real_position

        real_position = (real_position[0] - self.initial_robot_real_position[0] , \
                         real_position[1] - self.initial_robot_real_position[1])
        print(real_position)

        map_position = (real_position[0] / 0.05 , real_position[1] / 0.05)
        if len(self.trajectory_points) != 0:
            map_position = (int(map_position[0] + self.initial_robot_map_position[0])), \
                            int(map_position[1] * -1 + self.initial_robot_map_position[1])
            return map_position
        else:
            return None


    def rotate_point(self, x, y):
        x2 = x * math.cos(self.map_angle) - y * math.sin(self.map_angle)
        y2 = y * math.cos(self.map_angle) + x * math.sin(self.map_angle)
        return (x2, y2)


    def get_distance(self, x1, y1, x2, y2):
        return cv2.norm((x1, y1), (x2, y2))


    def get_angle(self, x1, y1, x2, y2):
        return math.atan2(y2 - y1, x2 - x1)


    def move_robot(self, x2, y2, theta):
        if self.robot_position != None:
            x1, y1 = self.robot_position
            distance = self.get_distance(x1, y1, x2, y2)
            angle = self.get_angle(x1, y1, x2, y2)

            dx = distance * math.cos(angle - self.map_angle) * self.pixel_distance
            dy = distance * math.sin(angle - self.map_angle) * self.pixel_distance * -1.0

            self.motion.moveTo(dx, dy, theta)

            print(str(self.robot_position) + " - " + str((x2, y2)) + ": " + str(dx) + " " + str(dy))


    def draw_trajectory(self):
        image = deepcopy(self.image)
        for i in range(len(self.trajectory_points)):
            image = cv2.circle(image, self.trajectory_points[i], 8, 12, -1)
        
        if len(self.trajectory_points) > 1:
            for i in range(1, len(self.trajectory_points)):
                point1 = self.trajectory_points[i-1]
                point2 = self.trajectory_points[i]
                image = cv2.line(image, point1, point2, 12, 1) 

        if self.robot_position != None:
            rotated_position = self.rotate_point(self.robot_position)
            image = cv2.circle(image, rotated_position, 8, 100, -1)
            if len(self.trajectory_points) > 0:
                image = cv2.line(image, self.robot_position, self.trajectory_points[0], 12, 1)

        return image


    def run(self):
        print("Running...")

        try:
            while True:
                self.robot_position = self.get_robot_position()

                key = cv2.waitKey(20)
                if key == ENTER_KEY_CODE:
                    if len(self.trajectory_points) > 1:
                        next_position = self.trajectory_points[1]
                        self.move_robot(next_position[0], next_position[1], 0)
                        self.trajectory_points.pop(0)

                if key == ESC_KEY_CODE:
                    break

                cv2.imshow("Map", self.draw_trajectory())
                cv2.resizeWindow("Map", 1024, 1024)

            cv2.destroyAllWindows()
                

        except KeyboardInterrupt:
            print("Script interrupted by user, shutting down.")
            cv2.destroyAllWindows()
            sys.exit(0)


if __name__ == "__main__":
    application = None
    
    # Initialize NAOqi framework.
    try:
        connection_url = "tcp://" + ip + ":" + str(port)
        application = qi.Application(["Main", "--qi-url=" + connection_url])
    except RuntimeError:
        print("Can't connect to Naoqi at \"" + ip + ":"  + str(port) +"\".\n"
            "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    main = Navigate(application)
    main.run()

    sys.path.remove(naoqi_path)
