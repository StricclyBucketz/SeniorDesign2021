import rospy
import enum
import sys
import numpy as np

from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs import msg
from dynamixel_workbench_msgs import srv

import navx_micro
from navx_micro._impl.imuregisters import IMURegisters
from navx_micro._impl.ahrsprotocol import AHRSProtocol
from geometry_msgs.msg import Quaternion, Vector3, Vector3Stamped
from sensor_msgs.msg import Imu

import functions.py

TOLERANCE = 3 # %tolerance = TOLERANCE / 1023 (In this case ~0.3%)
STAND = np.array([512,512,512,512,512,512,512,512,512,512,512,512])
RIGHT_FOOT_FORWARD = np.array([512,512,720,750,512,512,512,512,512,512,512,512])
RIGHT_FOOT_DOWN = np.array([512,512,720,750,512,512,512,405,455,565,512,512])
RIGHT_FOOT_TORSO = np.array([512,512,720,740,512,512,512,405,455,565,512,512])
RIGHT_LEG_STRAIGHT = np.array([512,512,512,512,512,512,512,405,455,565,512,512])
LEFT_FOOT_FORWARD = np.array([512,512,512,512,512,512,512,512,304,295,512,512])
LEFT_FOOT_DOWN = np.array([512,620,570,464,512,512,512,512,304,295,512,512])
LEFT_FOOT_TORSO = np.array([512,620,570,464,512,512,512,512,304,299,512,512])
LEFT_LEG_STRAIGHT = np.array([512,620,570,464,512,512,512,512,512,512,512,512])

class ForwardFoot(enum.Enum):
  neither = 0
  right = 1
  left = 2

# Takes a numpy.array of size 12.  Spins until position read in from servos are all within tolerance
# TODO replace servo#Pos with read ins from servos
def spinWhileMoving(targets):
	#call motor subscriber
  while(abs(servo1Pos - targets[0]) > TOLERANCE or abs(servo2Pos - targets[1]) > TOLERANCE or abs(servo3Pos - targets[2]) > TOLERANCE or abs(servo4Pos - targets[3]) > TOLERANCE or abs(servo5Pos - targets[4]) > TOLERANCE or abs(servo6Pos - targets[5]) > TOLERANCE or abs(servo7Pos - targets[6]) > TOLERANCE or abs(servo8Pos - targets[7]) > TOLERANCE or abs(servo9Pos - targets[8]) > TOLERANCE or abs(servo10Pos - targets[9]) > TOLERANCE or abs(servo11Pos - targets[10]) > TOLERANCE or abs(servo12Pos - targets[11]) > TOLERANCE):
    pass

def stepOff():
  # TODO tell manager to put servos in RIGHT_FOOT_FORWARD positions
  
  spinWhileMoving(RIGHT_FOOT_FORWARD)
  # TODO tell manager to put servos in RIGHT_FOOT_DOWN positions
  spinWhileMoving(RIGHT_FOOT_DOWN)
  # TODO tell manager to put servos in RIGHT_FOOT_TORSO positions
  spinWhileMoving(RIGHT_FOOT_TORSO) # could be replaced by righting IMU

def leftStep():
  # TODO tell manager to put servos in RIGHT_LEG_STRAIGHT postions
  spinWhileMoving(RIGHT_LEG_STRAIGHT)
  # TODO tell manager to put servos in LEFT_FOOT_FORWARD positions
  spinWhileMoving(LEFT_FOOT_FORWARD)
  # TODO tell manager to put servos in LEFT_FOOT_DOWN positions
  spinWhileMoving(LEFT_FOOT_DOWN)
  # TODO tell manager to put servos in LEFT_FOOT_TORSO positions
  spinWhileMoving(LEFT_FOOT_TORSO) # could be replaced by righting IMU

def rightStep():
  # TODO tell manager to put servos in LEFT_LEG_STRAIGHT positions
  spinWhileMoving(LEFT_LEG_STRAIGHT)
  # TODO tell manager to put servos in RIGHT_FOOT_FORWARD positions
  spinWhileMoving(RIGHT_FOOT_FORWARD)
  # TODO tell manager to put servos in RIGHT_FOOT_DOWN positions
  spinWhileMoving(RIGHT_FOOT_DOWN)
  # TODO tell manager to put servos in RIGHT_FOOT_TORSO positions
  spinWhileMoving(RIGHT_FOOT_TORSO) # could be replaced by righting IMU

def closeStep():
  # TODO tell manager to put servos in STAND positions
  spinWhileMoving(STAND)

def walkLogic(stepsToTake):
  stepsTaken = 0
  forwardFoot = ForwardFoot.neither
  while stepsTaken < stepsToTake:
    if forwardFoot == ForwardFoot.neither:
      print(f"StepOff\tstep#: {(stepsTaken+1)}")
      stepOff()
      forwardFoot = ForwardFoot.right
    elif forwardFoot == ForwardFoot.right:
      print(f"LeftStep\tstep#: {(stepsTaken+1)}")
      leftStep()
      forwardFoot = ForwardFoot.left
    elif forwardFoot == ForwardFoot.left:
      print(f"RightStep\tstep#: {(stepsTaken+1)}")
      forwardFoot = ForwardFoot.right
    else:
      sys.exit("ERROR: Invalid foot forward")
    stepsTaken += 1
  print("CloseStep")
  closeStep()

if __name__ == "__main__":
  walkLogic(5)
