
import zmq
import msgpack
import math

STD_TOPIC = "ESP/CTRL" # Default topic for ESP32 control messages
SPEED_UPDTE_CMD = "25" # Command to update speed in the ESP32
class ControlHolo:
    def __init__(self, socket, maxV: float, maxW: float, verbose=False):
        self.socket = socket    
        self.verbose = verbose

        self.robotV     = 0.0
        self.robotVn    = 0.0
        self.robotW     = 0.0

        self.maxRobotV  = maxV
        self.maxRobotW  = maxW

        self.robotX = 0.0
        self.robotY = 0.0
        self.robotTheta = 0.0

    def update(self):
        topic = STD_TOPIC
        data  = "<" + SPEED_UPDTE_CMD + f":{self.robotV},{self.robotVn},{self.robotW}\n"
        if self.verbose: print("[INFO] Sending:", data)
        self.socket.send_multipart([topic.encode(), msgpack.packb(data)])

    def setRobotPose(self, x, y, theta):
        if self.verbose: print(f"[INFO] Setting robot pose to ({x}, {y}, {theta})")
        self.robotX = x
        self.robotY = y
        self.robotTheta = theta

    def stopRobot(self):
        self.robotV = 0.0
        self.robotVn = 0.0
        self.robotW = 0.0

    def goToXYT(self, targetX, targetY, targetTheta):
        if self.verbose: print(f"[INFO] Moving to ({targetX}, {targetY}) from ({self.robotX}, {self.robotY})")
        
        A = 1
        B = 2

        # ANGULAR VELOCITY
        dTheta = targetTheta - self.robotTheta
        if dTheta > math.pi():
            dTheta -= 2 * math.pi
        elif dTheta < -math.pi:
            dTheta += 2 * math.pi

        if abs(dTheta) > 0.1:
            W = B * dTheta  
            if W > self.maxRobotW:
                W = self.maxRobotW
            self.robotW = W
        else:
            self.robotW = 0.0

        # LINEAR VELOCITY
        dx = targetX - self.robotX
        dy = targetY - self.robotY
        distance = (dx**2 + dy**2)**0.5

        angle = math.atan2(dy, dx)
        angle = angle - self.robotTheta
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi

        Vf =  A * distance
        if Vf > self.maxRobotV:
            Vf = self.maxRobotV

        if distance > 0.05:
            self.robotV  = math.cos(angle) * Vf
            self.robotVn = math.sin(angle) * Vf
        else:
            self.robotV = 0.0
            self.robotVn = 0.0


    def blindForward(self, speed):
        self.robotV  = speed
        self.robotVn = 0.0
        self.robotW  = 0.362

    def blindBackward(self, speed):
        self.robotV  = -speed
        self.robotVn = 0.0
        self.robotW  = 0.0
    
    def blindRight(self, speed):
        self.robotV  = 0.0
        self.robotVn = -speed
        self.robotW  = 0.0

    def blindLeft(self, speed):
        self.robotV  = 0.0
        self.robotVn = speed
        self.robotW  = 0.0
    
    def blindTurnRight(self, speed):
        self.robotV  = 0.0
        self.robotVn = 0.0
        self.robotW  = -speed

    def blindTurnLeft(self, speed):
        self.robotV  = 0.0
        self.robotVn = 0.0
        self.robotW  = speed