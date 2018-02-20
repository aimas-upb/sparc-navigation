import sys
naoqi_path = "/home/sparc-308/workspace/stefania/pynaoqi-python2.7/lib/python2.7/site-packages"
sys.path.insert(0, naoqi_path)
import cv2
import qi
import math

robotPosition = (-1,-1)
initialRobotMapPosition = (-1, -1)
initialRobotRealPosition = (-1000, -1000)

initialImage = cv2.imread("map.png", 0)
image = cv2.imread("map.png", 0)
wayPoints = []
ip = "192.168.0.115"
port = 9559
session = qi.Session()
navigation_service = None
motion_service = None

def onMouse(event, x, y, c, w):
    global initialRobotMapPosition
    if (event == cv2.EVENT_LBUTTONDOWN):
        val = initialImage[y][x]
        wayPoints.append((x, y))
        if initialRobotMapPosition[0] == -1:
        	initialRobotMapPosition = (x,y)



    elif (event == cv2.EVENT_MBUTTONDOWN):
        robotPosition = (x, y)
    elif event == cv2.EVENT_RBUTTONDOWN:
        for i in range(len(wayPoints)):
            wayPoint = wayPoints[i]
            res = cv2.norm(wayPoint, (x, y))
            if (res < 20.0):
                wayPoints.remove(i)

def getRobotPosition():
    global initialRobotMapPosition
    global initialRobotRealPosition

    result = motion_service.getRobotPosition(True)
    if(initialRobotRealPosition[0] == -1000):
    	initialRobotRealPosition = result

    result = (result[0] - initialRobotRealPosition[0] , result[1] - initialRobotRealPosition[1])
    print(result)
    

    result = (result[0] / 0.05 , result[1] / 0.05)

    if len(wayPoints) > 0:
    	return (int(result[0] + initialRobotMapPosition[0]), int(-1 * result[1] + initialRobotMapPosition[1]))
    else:
    	return (-1, -1)

def drawWaypoints():
    from copy import deepcopy
    global image
    image = deepcopy(initialImage)
    for i in range(len(wayPoints)):
        waypoint = wayPoints[i]
        image = cv2.circle(image, waypoint, 8, 12, -1)
    if len(wayPoints) > 1:
	    for i in range(1, len(wayPoints)):
	        waypoint1 = wayPoints[i - 1]
	        waypoint2 = wayPoints[i]
	        image = cv2.line(image, waypoint1, waypoint2, 12, 1)
	         
	         

    if robotPosition[0] != -1 and robotPosition[1] != -1:
        image = cv2.circle(image, robotPosition, 8, 100, -1)
        if len(wayPoints) > 0:
            image = cv2.line(image, robotPosition, wayPoints[0], 12, 1)

def getAngle(x1,y1, x2,y2, d):
	angle = math.atan2(y2-y1, x2-x1)
	print(math.degrees(angle))
	return angle


def moveRobot(x2, y2, theta2):
	global robotPosition
	dalpha = -0.23404

	x1, y1 = robotPosition
	if x1 != -1 and y1 != -1:
		d = cv2.norm((x1,y1), (x2,y2)) 
		print(str((x2,y2)) + " " + str(robotPosition) + " "  + str(d))
		alpha = getAngle(x1,y1,x2,y2,d)
		print(math.degrees(alpha))

		dx = d * math.cos(alpha - dalpha) * 0.05
		dy = d * math.sin(alpha - dalpha) * 0.05 * -1.0
		print(str(dx) + " " + str(dy))
		motion_service.moveTo(dx, dy, theta2)
		wayPoints.remove(wayPoints[0])



try:
	session.connect("tcp://" + ip + ":" + str(port))
	navigation_service = session.service("ALNavigation")
	motion_service = session.service("ALMotion")
	motion_service.moveInit()
	# Wake up robot
	motion_service.wakeUp()
except RuntimeError:
	print ("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) +".\n"
		"Please check your script arguments. Run with -h option for help.")
	sys.exit(1)


if image.data:
    print("Could not open or find the image")

cv2.namedWindow( "image", cv2.WINDOW_NORMAL )
cv2.setMouseCallback( "image", onMouse, 0 )

while True:
    robotPosition = getRobotPosition()
    drawWaypoints()
    c = cv2.waitKey(20)
    if(c == 27):
        break
    if(c == 13):
        moveRobot(wayPoints[1][0],wayPoints[1][1], 0)
    cv2.imshow( "image", image)
    cv2.resizeWindow("image", 1024, 1024)
cv2.destroyAllWindows()

sys.path.remove(naoqi_path)