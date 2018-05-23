from naoqi import ALProxy
from flask import Flask

class PepperMovement():
    def __init__(self, ip, port = 9559):
        self.ip = ip
        self.port = port
        self.x = 0.0
        self.y = 0.0
        self.movement = ALProxy("ALMotion", self.ip, self.port)

    def moveForward(self):
        self.x = self.x + 0.1
        self.movement.moveTo(0.1, self.y, 0)
        print("Moved to " + str(self.x))

    def moveBackward(self):
        self.x = self.x - 0.1
        self.movement.moveTo(-0.1, self.y, 0)
        print("Moved to " + str(self.x))

app = Flask(__name__)
pepper = PepperMovement("192.168.0.115")

@app.route("/moveForward", methods=["GET"])
def moveForward():
    pepper.moveForward()
    return "Done"

@app.route("/moveBackward", methods=["GET"])
def moveBackward():
    pepper.moveBackward()
    return "Done"


if __name__ == '__main__':
    app.run(host= '0.0.0.0')