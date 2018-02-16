import cv2

robotPosition = (-1,-1)
initialImage = cv2.imread("C:\\Users\\AlexandruGAVRIL\\build-SparcNavigation-Desktop_Qt_5_7_1_MSVC2015_64bit-Release\\release\\mergeee.map.png", 0)
image = cv2.imread("C:\\Users\\AlexandruGAVRIL\\build-SparcNavigation-Desktop_Qt_5_7_1_MSVC2015_64bit-Release\\release\\mergeee.map.png", 0)
wayPoints = []

def onMouse(event, x, y, c, w):
    if (event == cv2.EVENT_LBUTTONDOWN):
        val = initialImage[y][x]
        print(str(x) + " " + str(y) + " " + str(val))
        wayPoints.append((x, y))
    elif (event == cv2.EVENT_MBUTTONDOWN):
        robotPosition = (x, y)
    elif event == cv2.EVENT_RBUTTONDOWN:
        for i in range(len(wayPoints)):
            wayPoint = wayPoints[i]
            res = cv2.norm(wayPoint, (x, y))
            if (res < 20.0):
                wayPoints.remove(i)

def getRobotPosition():
    pass

def drawWaypoints():
    print(wayPoints)
    from copy import deepcopy
    global image
    image = deepcopy(initialImage)
    for i in range(len(wayPoints)):
        waypoint = wayPoints[i]
        print(cv2.circle(image, waypoint, 8, 12, -1))
        image = cv2.circle(image, waypoint, 8, 12, -1)
    for i in range(len(wayPoints)):
        waypoint1 = wayPoints[i - 1]
        waypoint2 = wayPoints[i]
        image = cv2.line(image, waypoint1, waypoint2, 12, 1)
    if robotPosition[0] != -1 and robotPosition[1] != -1:
        image = cv2.circle(image, robotPosition, 8, 100, -1)
        if len(wayPoints) > 0:
            image = cv2.line(image, robotPosition, wayPoints[0], 12, 1)

def moveRobot():
    pass

if image.data:
    print("Could not open or find the image")

cv2.namedWindow( "image", cv2.WINDOW_NORMAL )
cv2.setMouseCallback( "image", onMouse, 0 )
while True:
    getRobotPosition()
    drawWaypoints()

    c = cv2.waitKey(20)
    if(c == 27):
        break
    if(c == 13):
        moveRobot()
    cv2.imshow( "image", image)
    cv2.resizeWindow("image", 1024, 1024)
cv2.destroyAllWindows()

