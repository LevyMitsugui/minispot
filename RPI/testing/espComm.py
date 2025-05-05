import serial
import time
import sys

import numpy as np
from spotmicro.Kinematics.LegKinematics import LegIK
from spotmicro.Kinematics.LieAlgebra import RpToTrans, TransToRp, TransInv, RPY, TransformVector
from collections import OrderedDict

from Kinematics.SpotKinematics import SpotModel

# model = SpotModel(
#     shoulder_length=your_value,
#     elbow_length=your_value,
#     wrist_length=your_value
# )

#model = SpotModel()
PI = np.pi

model = SpotModel(
    shoulder_length=0.045,
    elbow_length=   0.08,
    wrist_length=   0.103,
    hip_x=0.185,        # front-back distance between hips
    hip_y=0.077,        # side-to-side distance between hips
    foot_x=0.185,       # front-back distance between feet
    foot_y=0.17,       # side-to-side distance between feet
    height=0.145        # vertical body-to-ground distance
)


model.pf_FL = np.array([ 0.0925,  0.085, -0.145])
model.pf_FR = np.array([ 0.0925, -0.085, -0.145])
model.pf_BL = np.array([-0.0925,  0.085, -0.145])
model.pf_BR = np.array([-0.0925, -0.085, -0.145])

model.WorldToFoot["FL"] = RpToTrans(np.eye(3), model.pf_FL)
model.WorldToFoot["FR"] = RpToTrans(np.eye(3), model.pf_FR)
model.WorldToFoot["BL"] = RpToTrans(np.eye(3), model.pf_BL)
model.WorldToFoot["BR"] = RpToTrans(np.eye(3), model.pf_BR)

roll = 0.0
pitch = 0.52
yaw = 0

orn = np.array([roll, pitch, yaw])  # e.g., [0.1, -0.2, 0.0]
pos = np.array([0.0, 0.0, 0.0])    # Raise torso by 12 cm
T_bf = model.WorldToFoot            # Default foot positions
joint_angles = model.IK(orn, pos, T_bf) #in radians
joint_angles = np.degrees(joint_angles)
print(joint_angles)

speed = 256
try:
    esp_serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
except (serial.SerialException, OSError) as e:
    print(f"[ERROR] Serial port error: {e}")
    sys.exit(1)

msg = "<0:"
for joint in joint_angles:
    for pos in joint:
        msg += "{:.3f},".format(pos)
        msg += "{:.3f},".format(speed)
msg = msg[:-1]
msg+="\n"
print(msg)
stance = "<2\n"
esp_serial.write(stance.encode())
esp_serial.flush()

input("Press enter to continue...")
esp_serial.write(msg.encode())
esp_serial.flush()

input("Press enter to continue...")
esp_serial.write(stance.encode())
esp_serial.flush()