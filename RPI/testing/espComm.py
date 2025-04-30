import serial
import time

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


model.pf_FL = np.array([ 0.06,  0.04, -0.10])
model.pf_FR = np.array([ 0.06, -0.04, -0.10])
model.pf_BL = np.array([-0.06,  0.04, -0.10])
model.pf_BR = np.array([-0.06, -0.04, -0.10])

model.WorldToFoot["FL"] = RpToTrans(np.eye(3), model.pf_FL)
model.WorldToFoot["FR"] = RpToTrans(np.eye(3), model.pf_FR)
model.WorldToFoot["BL"] = RpToTrans(np.eye(3), model.pf_BL)
model.WorldToFoot["BR"] = RpToTrans(np.eye(3), model.pf_BR)


row = 0
roll = 0
pitch = 0
yaw = 0

orn = np.array([roll, pitch, yaw])  # e.g., [0.1, -0.2, 0.0]
pos = np.array([0.0, 0.0, 0.0])    # Raise torso by 12 cm
T_bf = model.WorldToFoot            # Default foot positions
joint_angles = model.IK(orn, pos, T_bf) #in radians
joint_angles = np.degrees(joint_angles)
print(joint_angles)
