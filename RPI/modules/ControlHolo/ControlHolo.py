
import zmq
import msgpack

STD_TOPIC = "ESP/CTRL" # Default topic for ESP32 control messages
SPEED_UPDTE_CMD = "25" # Command to update speed in the ESP32
class ControlHolo:
    def __init__(self, socket, verbose=False):
        self.socket = socket    
        self.verbose = verbose

        self.robotV  = 0.0
        self.robotVn = 0.0
        self.robotW  = 0.0

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

    def goToXYT(self, targetX, targetY, targetTheta):
        if self.verbose: print(f"[INFO] Moving to ({targetX}, {targetY}) from ({self.robotX}, {self.robotY})")
        
        A = 1
        B = 2

        # ANGULAR VELOCITY
        dTheta = targetTheta - self.robotTheta
        if dTheta > pi:
            dTheta -= 2 * pi
        elif dTheta < -pi:
            dTheta += 2 * pi

        if abs(dTheta) > 0.1:
            self.robotW = B * dTheta
        else:
            self.robotW = 0.0

        # LINEAR VELOCITY
        dx = targetX - self.robotX
        dy = targetY - self.robotY
        distance = (dx**2 + dy**2)**0.5

        angle = atan2(dy, dx)
        angle = angle - self.robotTheta
        if angle > pi:
            angle -= 2 * pi
        elif angle < -pi:
            angle += 2 * pi

        Vf =  A * distance

        if distance > 0.05:
            self.robotV  = cos(angle) * Vf
            self.robotVn = sin(angle) * Vf
        else:
            self.robotV = 0.0
            self.robotVn = 0.0


    def blindForward(self, speed):
        self.robotV  = speed
        self.robotVn = 0.0
        self.robotW  = 0.0

    def blindBackward(self, speed):
        self.robotV  = -speed
        self.robotVn = 0.0
        self.robotW  = 0.0
    
    def blindRight(self, speed):
        self.robotV  = 0.0
        self.robotVn = speed
        self.robotW  = 0.0

    def blindLeft(self, speed):
        self.robotV  = 0.0
        self.robotVn = -speed
        self.robotW  = 0.0
    
    def blindTurnRight(self, speed):
        self.robotV  = 0.0
        self.robotVn = 0.0
        self.robotW  = -speed

    def blindTurnLeft(self, speed):
        self.robotV  = 0.0
        self.robotVn = 0.0
        self.robotW  = speed