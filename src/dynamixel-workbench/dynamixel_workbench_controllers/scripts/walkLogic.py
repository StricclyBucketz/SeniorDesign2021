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

def callback();

# Takes a numpy.array of size 12.  Spins until position read in from servos are all within tolerance
# TODO replace servo#Pos with read ins from servos
def spinWhileMoving(targets):
	#call motor subscriber
    rospy.init_node('motorSub', anonymous=True)
    posSub = rospy.Subscriber("dynamixel_workbench/dynamixel_state", msg.DynamixelStateList, callback)
    #rospy.spin()
    #TODO find the indeces of the motors in DynamixelStateList and populate below
    #Christian will do this when we can test
  while(abs(servo1Pos - targets[0]) > TOLERANCE or abs(servo2Pos - targets[1]) > TOLERANCE or abs(servo3Pos - targets[2]) > TOLERANCE or abs(servo4Pos - targets[3]) > TOLERANCE or abs(servo5Pos - targets[4]) > TOLERANCE or abs(servo6Pos - targets[5]) > TOLERANCE or abs(servo7Pos - targets[6]) > TOLERANCE or abs(servo8Pos - targets[7]) > TOLERANCE or abs(servo9Pos - targets[8]) > TOLERANCE or abs(servo10Pos - targets[9]) > TOLERANCE or abs(servo11Pos - targets[10]) > TOLERANCE or abs(servo12Pos - targets[11]) > TOLERANCE):
    pass

def stepOff():
  # TODO Also determine if we can get away with only calling the motors that need to move for each step instead of all of them
  # TODO test and see if we need to implement time.sleep() in between certain lines
  # TODO get speed values for all of these control functions
  speed_pos_control(1, 512, RIGHT_FOOT_FORWARD[0])
  speed_pos_control(2, 512, RIGHT_FOOT_FORWARD[1])
  speed_pos_control(3, 512, RIGHT_FOOT_FORWARD[2])
  speed_pos_control(4, 512, RIGHT_FOOT_FORWARD[3])
  speed_pos_control(5, 512, RIGHT_FOOT_FORWARD[4])
  speed_pos_control(6, 512, RIGHT_FOOT_FORWARD[5])
  speed_pos_control(7, 512, RIGHT_FOOT_FORWARD[6])
  speed_pos_control(8, 512, RIGHT_FOOT_FORWARD[7])
  speed_pos_control(9, 512, RIGHT_FOOT_FORWARD[8])
  speed_pos_control(10, 512, RIGHT_FOOT_FORWARD[9])
  speed_pos_control(11, 512, RIGHT_FOOT_FORWARD[10])
  speed_pos_control(12, 512, RIGHT_FOOT_FORWARD[11])
  spinWhileMoving(RIGHT_FOOT_FORWARD)
  # TODO Speeds
  speed_pos_control(1, 512, RIGHT_FOOT_DOWN[0])
  speed_pos_control(2, 512, RIGHT_FOOT_DOWN[1])
  speed_pos_control(3, 512, RIGHT_FOOT_DOWN[2])
  speed_pos_control(4, 512, RIGHT_FOOT_DOWN[3])
  speed_pos_control(5, 512, RIGHT_FOOT_DOWN[4])
  speed_pos_control(6, 512, RIGHT_FOOT_DOWN[5])
  speed_pos_control(7, 512, RIGHT_FOOT_DOWN[6])
  speed_pos_control(8, 512, RIGHT_FOOT_DOWN[7])
  speed_pos_control(9, 512, RIGHT_FOOT_DOWN[8])
  speed_pos_control(10, 512, RIGHT_FOOT_DOWN[9])
  speed_pos_control(11, 512, RIGHT_FOOT_DOWN[10])
  speed_pos_control(12, 512, RIGHT_FOOT_DOWN[11])
  spinWhileMoving(RIGHT_FOOT_DOWN)
  # TODO Speeds
  speed_pos_control(1, 512, RIGHT_FOOT_TORSO[0])
  speed_pos_control(2, 512, RIGHT_FOOT_TORSO[1])
  speed_pos_control(3, 512, RIGHT_FOOT_TORSO[2])
  speed_pos_control(4, 512, RIGHT_FOOT_TORSO[3])
  speed_pos_control(5, 512, RIGHT_FOOT_TORSO[4])
  speed_pos_control(6, 512, RIGHT_FOOT_TORSO[5])
  speed_pos_control(7, 512, RIGHT_FOOT_TORSO[6])
  speed_pos_control(8, 512, RIGHT_FOOT_TORSO[7])
  speed_pos_control(9, 512, RIGHT_FOOT_TORSO[8])
  speed_pos_control(10, 512, RIGHT_FOOT_TORSO[9])
  speed_pos_control(11, 512, RIGHT_FOOT_TORSO[10])
  speed_pos_control(12, 512, RIGHT_FOOT_TORSO[11])
  spinWhileMoving(RIGHT_FOOT_TORSO) # could be replaced by righting IMU
  #Replacing with IMU measurements would look something like this:
  #rospy.init_node('imu_Node', anonymous=True)
  #sub = rospy.Subscriber("navx_micro/euler", Vector3Stamped, callback)
  #pitch = sub.vector.x
  #roll = sub.vector.y
  #yaw = sub.vector.z
  #Then implement a spin function like you were using, could make the callback
  #function I have blank for do that, but we would need separate for the
  #motors and the IMU
def leftStep():
  # TODO Speeds
  speed_pos_control(1, 512, RIGHT_LEG_STRAIGHT[0])
  speed_pos_control(2, 512, RIGHT_LEG_STRAIGHT[1])
  speed_pos_control(3, 512, RIGHT_LEG_STRAIGHT[2])
  speed_pos_control(4, 512, RIGHT_LEG_STRAIGHT[3])
  speed_pos_control(5, 512, RIGHT_LEG_STRAIGHT[4])
  speed_pos_control(6, 512, RIGHT_LEG_STRAIGHT[5])
  speed_pos_control(7, 512, RIGHT_LEG_STRAIGHT[6])
  speed_pos_control(8, 512, RIGHT_LEG_STRAIGHT[7])
  speed_pos_control(9, 512, RIGHT_LEG_STRAIGHT[8])
  speed_pos_control(10, 512, RIGHT_LEG_STRAIGHT[9])
  speed_pos_control(11, 512, RIGHT_LEG_STRAIGHT[10])
  speed_pos_control(12, 512, RIGHT_LEG_STRAIGHT[11])
  spinWhileMoving(RIGHT_LEG_STRAIGHT)
  # TODO Speeds
  speed_pos_control(1, 512, LEFT_FOOT_FORWARD[0])
  speed_pos_control(2, 512, LEFT_FOOT_FORWARD[1])
  speed_pos_control(3, 512, LEFT_FOOT_FORWARD[2])
  speed_pos_control(4, 512, LEFT_FOOT_FORWARD[3])
  speed_pos_control(5, 512, LEFT_FOOT_FORWARD[4])
  speed_pos_control(6, 512, LEFT_FOOT_FORWARD[5])
  speed_pos_control(7, 512, LEFT_FOOT_FORWARD[6])
  speed_pos_control(8, 512, LEFT_FOOT_FORWARD[7])
  speed_pos_control(9, 512, LEFT_FOOT_FORWARD[8])
  speed_pos_control(10, 512, LEFT_FOOT_FORWARD[9])
  speed_pos_control(11, 512, LEFT_FOOT_FORWARD[10])
  speed_pos_control(12, 512, LEFT_FOOT_FORWARD[11])
  spinWhileMoving(LEFT_FOOT_FORWARD)
  # TODO Speeds
  speed_pos_control(1, 512, LEFT_FOOT_DOWN[0])
  speed_pos_control(2, 512, LEFT_FOOT_DOWN[1])
  speed_pos_control(3, 512, LEFT_FOOT_DOWN[2])
  speed_pos_control(4, 512, LEFT_FOOT_DOWN[3])
  speed_pos_control(5, 512, LEFT_FOOT_DOWN[4])
  speed_pos_control(6, 512, LEFT_FOOT_DOWN[5])
  speed_pos_control(7, 512, LEFT_FOOT_DOWN[6])
  speed_pos_control(8, 512, LEFT_FOOT_DOWN[7])
  speed_pos_control(9, 512, LEFT_FOOT_DOWN[8])
  speed_pos_control(10, 512, LEFT_FOOT_DOWN[9])
  speed_pos_control(11, 512, LEFT_FOOT_DOWN[10])
  speed_pos_control(12, 512, LEFT_FOOT_DOWN[11])
  spinWhileMoving(LEFT_FOOT_DOWN)
  # TODO Speeds
  speed_pos_control(1, 512, LEFT_FOOT_TORSO[0])
  speed_pos_control(2, 512, LEFT_FOOT_TORSO[1])
  speed_pos_control(3, 512, LEFT_FOOT_TORSO[2])
  speed_pos_control(4, 512, LEFT_FOOT_TORSO[3])
  speed_pos_control(5, 512, LEFT_FOOT_TORSO[4])
  speed_pos_control(6, 512, LEFT_FOOT_TORSO[5])
  speed_pos_control(7, 512, LEFT_FOOT_TORSO[6])
  speed_pos_control(8, 512, LEFT_FOOT_TORSO[7])
  speed_pos_control(9, 512, LEFT_FOOT_TORSO[8])
  speed_pos_control(10, 512, LEFT_FOOT_TORSO[9])
  speed_pos_control(11, 512, LEFT_FOOT_TORSO[10])
  speed_pos_control(12, 512, LEFT_FOOT_TORSO[11])
  spinWhileMoving(LEFT_FOOT_TORSO) # could be replaced by righting IMU
  #Replacing with IMU measurements would look something like this:
  #rospy.init_node('imu_Node', anonymous=True)
  #sub = rospy.Subscriber("navx_micro/euler", Vector3Stamped, callback)
  #pitch = sub.vector.x
  #roll = sub.vector.y
  #yaw = sub.vector.z
  #Then implement a spin function like you were using, could make the callback
  #function I have blank for do that, but we would need separate for the
  #motors and the IMU

def rightStep():
  # TODO Speeds
  speed_pos_control(1, 512, LEFT_LEG_STRAIGHT[0])
  speed_pos_control(2, 512, LEFT_LEG_STRAIGHT[1])
  speed_pos_control(3, 512, LEFT_LEG_STRAIGHT[2])
  speed_pos_control(4, 512, LEFT_LEG_STRAIGHT[3])
  speed_pos_control(5, 512, LEFT_LEG_STRAIGHT[4])
  speed_pos_control(6, 512, LEFT_LEG_STRAIGHT[5])
  speed_pos_control(7, 512, LEFT_LEG_STRAIGHT[6])
  speed_pos_control(8, 512, LEFT_LEG_STRAIGHT[7])
  speed_pos_control(9, 512, LEFT_LEG_STRAIGHT[8])
  speed_pos_control(10, 512, LEFT_LEG_STRAIGHT[9])
  speed_pos_control(11, 512, LEFT_LEG_STRAIGHT[10])
  speed_pos_control(12, 512, LEFT_LEG_STRAIGHT[11])
  spinWhileMoving(LEFT_LEG_STRAIGHT)
  # TODO Speeds
  speed_pos_control(1, 512, RIGHT_FOOT_FORWARD[0])
  speed_pos_control(2, 512, RIGHT_FOOT_FORWARD[1])
  speed_pos_control(3, 512, RIGHT_FOOT_FORWARD[2])
  speed_pos_control(4, 512, RIGHT_FOOT_FORWARD[3])
  speed_pos_control(5, 512, RIGHT_FOOT_FORWARD[4])
  speed_pos_control(6, 512, RIGHT_FOOT_FORWARD[5])
  speed_pos_control(7, 512, RIGHT_FOOT_FORWARD[6])
  speed_pos_control(8, 512, RIGHT_FOOT_FORWARD[7])
  speed_pos_control(9, 512, RIGHT_FOOT_FORWARD[8])
  speed_pos_control(10, 512, RIGHT_FOOT_FORWARD[9])
  speed_pos_control(11, 512, RIGHT_FOOT_FORWARD[10])
  speed_pos_control(12, 512, RIGHT_FOOT_FORWARD[11])
  spinWhileMoving(RIGHT_FOOT_FORWARD)
  # TODO Speeds
  speed_pos_control(1, 512, RIGHT_FOOT_DOWN[0])
  speed_pos_control(2, 512, RIGHT_FOOT_DOWN[1])
  speed_pos_control(3, 512, RIGHT_FOOT_DOWN[2])
  speed_pos_control(4, 512, RIGHT_FOOT_DOWN[3])
  speed_pos_control(5, 512, RIGHT_FOOT_DOWN[4])
  speed_pos_control(6, 512, RIGHT_FOOT_DOWN[5])
  speed_pos_control(7, 512, RIGHT_FOOT_DOWN[6])
  speed_pos_control(8, 512, RIGHT_FOOT_DOWN[7])
  speed_pos_control(9, 512, RIGHT_FOOT_DOWN[8])
  speed_pos_control(10, 512, RIGHT_FOOT_DOWN[9])
  speed_pos_control(11, 512, RIGHT_FOOT_DOWN[10])
  speed_pos_control(12, 512, RIGHT_FOOT_DOWN[11])
  spinWhileMoving(RIGHT_FOOT_DOWN)
  # TODO tell manager to put servos in RIGHT_FOOT_TORSO positions
  speed_pos_control(1, 512, RIGHT_FOOT_TORSO[0])
  speed_pos_control(2, 512, RIGHT_FOOT_TORSO[1])
  speed_pos_control(3, 512, RIGHT_FOOT_TORSO[2])
  speed_pos_control(4, 512, RIGHT_FOOT_TORSO[3])
  speed_pos_control(5, 512, RIGHT_FOOT_TORSO[4])
  speed_pos_control(6, 512, RIGHT_FOOT_TORSO[5])
  speed_pos_control(7, 512, RIGHT_FOOT_TORSO[6])
  speed_pos_control(8, 512, RIGHT_FOOT_TORSO[7])
  speed_pos_control(9, 512, RIGHT_FOOT_TORSO[8])
  speed_pos_control(10, 512, RIGHT_FOOT_TORSO[9])
  speed_pos_control(11, 512, RIGHT_FOOT_TORSO[10])
  speed_pos_control(12, 512, RIGHT_FOOT_TORSO[11])
  spinWhileMoving(RIGHT_FOOT_TORSO) # could be replaced by righting IMU
  #Replacing with IMU measurements would look something like this:
  #rospy.init_node('imu_Node', anonymous=True)
  #sub = rospy.Subscriber("navx_micro/euler", Vector3Stamped, callback)
  #pitch = sub.vector.x
  #roll = sub.vector.y
  #yaw = sub.vector.z
  #Then implement a spin function like you were using, could make the callback
  #function I have blank for do that, but we would need separate for the
  #motors and the IMU

def closeStep():
  # TODO tell manager to put servos in STAND positions
  speed_pos_control(1, 512, STAND[0])
  speed_pos_control(2, 512, STAND[1])
  speed_pos_control(3, 512, STAND[2])
  speed_pos_control(4, 512, STAND[3])
  speed_pos_control(5, 512, STAND[4])
  speed_pos_control(6, 512, STAND[5])
  speed_pos_control(7, 512, STAND[6])
  speed_pos_control(8, 512, STAND[7])
  speed_pos_control(9, 512, STAND[8])
  speed_pos_control(10, 512, STAND[9])
  speed_pos_control(11, 512, STAND[10])
  speed_pos_control(12, 512, STAND[11])
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
