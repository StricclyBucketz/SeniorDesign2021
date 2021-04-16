import rospy
import enum
import sys
import numpy as np
import math

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
BASE_MOTOR_SPEED = 512 # Sets the speed for the servo moving the farthest in a state change.
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

def calculateServoSpeed(targets):
  # loop through each servo, recording (in array) difference between each current position and target position (delta)
  # find greatestDelta
  deltas = targets
  maxDelta = -1024
  for delta in deltas:
    delta = abs(delta - servoPos) #TODO replace servoPos with read in from the appropriate servo.  Could be indexing into an array created prior to this loop
    if delta > maxDelta:
      maxDelta = delta
  for delta in deltas:
    delta = math.ceil(BASE_MOTOR_SPEED * delta/maxDelta)
  # return cieling(delta/greatestDelta)
  return deltas

def readInPositions():
  #arr = numpy.zeros([12])
  # servoInfos = array of length 12 (or 16/18) containing servoInfo objects read in from ROS
  #   servoInfo has fields: ID (unique ID for servo ranging from 1-12), Position (position of servo ranging from 0-1023) and potentially others.
  rospy.init_node('motorSub', anonymous=True)
  posSub = rospy.Subscriber("dynamixel_workbench/dynamixel_state", msg.DynamixelStateList, callback)

  arr = numpy.zeros(len(motorSub.dynamixel_state))
  for dynamixel_state in motorSub:
    arr[motorSub.dynamixel_state.id - 1] = motorSub.dynamixel_state.present_position
  return arr

def servoInfoIndexTest():
  # servoInfos read in via ROS
  rospy.init_node('motorSub', anonymous=True)
  posSub = rospy.Subscriber("dynamixel_workbench/dynamixel_state", msg.DynamixelStateList, callback)
  for dynamixel_state in motorSub:
    print("Servo {} is at position: {}".format(motorSub.dynamixel_state.id, motorSub.dynamixel_state.present_position))

# Takes a numpy.array of size 12.  Spins until position read in from servos are all within tolerance
# TODO replace servo#Pos with read ins from servos
def spinWhileMoving(targets, arr):
	#call motor subscriber
  rospy.init_node('motorSub', anonymous=True)
  posSub = rospy.Subscriber("dynamixel_workbench/dynamixel_state", msg.DynamixelStateList, callback)
  arr = readInPositions();
  #rospy.spin()
  #TODO find the indeces of the motors in DynamixelStateList and populate below
  #Christian will do this when we can test
  #while(abs(arr[0] - targets[0]) > TOLERANCE or abs(arr[1] - targets[1]) > TOLERANCE or abs(arr[2] - targets[2]) > TOLERANCE or abs(arr[3] - targets[3]) > TOLERANCE or abs(arr[4] - targets[4]) > TOLERANCE or abs(arr[5] - targets[5]) > TOLERANCE or abs(arr[6] - targets[6]) > TOLERANCE or abs(arr[7] - targets[7]) > TOLERANCE or abs(arr[8] - targets[8]) > TOLERANCE or abs(arr[9] - targets[9]) > TOLERANCE or abs(arr[10] - targets[10]) > TOLERANCE or abs(arr[11] - targets[11]) > TOLERANCE):
    #pass
  while(abs(arr[0] - targets[0]) > TOLERANCE or abs(arr[1] - targets[1]) > TOLERANCE or abs(arr[2] - targets[2]) > TOLERANCE or abs(arr[3] - targets[3]) > TOLERANCE or abs(arr[4] - targets[4]) > TOLERANCE or abs(arr[5] - targets[5]) > TOLERANCE or abs(arr[6] - targets[6]) > TOLERANCE or abs(arr[7] - targets[7]) > TOLERANCE or abs(arr[8] - targets[8]) > TOLERANCE or abs(arr[9] - targets[9]) > TOLERANCE or abs(arr[10] - targets[10]) > TOLERANCE or abs(arr[11] - targets[11]) > TOLERANCE):
    pass

def stepOff():
  # TODO Also determine if we can get away with only calling the motors that need to move for each step instead of all of them
  # TODO test and see if we need to implement time.sleep() in between certain lines
  speeds = calculateServoSpeed(RIGHT_FOOT_FORWARD)
  speed_pos_control(1, speeds[0], RIGHT_FOOT_FORWARD[0])
  speed_pos_control(2, speeds[1], RIGHT_FOOT_FORWARD[1])
  speed_pos_control(3, speeds[2], RIGHT_FOOT_FORWARD[2])
  speed_pos_control(4, speeds[3], RIGHT_FOOT_FORWARD[3])
  speed_pos_control(5, speeds[4], RIGHT_FOOT_FORWARD[4])
  speed_pos_control(6, speeds[5], RIGHT_FOOT_FORWARD[5])
  speed_pos_control(7, speeds[6], RIGHT_FOOT_FORWARD[6])
  speed_pos_control(8, speeds[7], RIGHT_FOOT_FORWARD[7])
  speed_pos_control(9, speeds[8], RIGHT_FOOT_FORWARD[8])
  speed_pos_control(10, speeds[9], RIGHT_FOOT_FORWARD[9])
  speed_pos_control(11, speeds[10], RIGHT_FOOT_FORWARD[10])
  speed_pos_control(12, speeds[11], RIGHT_FOOT_FORWARD[11])
  spinWhileMoving(RIGHT_FOOT_FORWARD)
  speeds = calculateServoSpeed(RIGHT_FOOT_DOWN)
  speed_pos_control(1, speeds[0], RIGHT_FOOT_DOWN[0])
  speed_pos_control(2, speeds[1], RIGHT_FOOT_DOWN[1])
  speed_pos_control(3, speeds[2], RIGHT_FOOT_DOWN[2])
  speed_pos_control(4, speeds[3], RIGHT_FOOT_DOWN[3])
  speed_pos_control(5, speeds[4], RIGHT_FOOT_DOWN[4])
  speed_pos_control(6, speeds[5], RIGHT_FOOT_DOWN[5])
  speed_pos_control(7, speeds[6], RIGHT_FOOT_DOWN[6])
  speed_pos_control(8, speeds[7], RIGHT_FOOT_DOWN[7])
  speed_pos_control(9, speeds[8], RIGHT_FOOT_DOWN[8])
  speed_pos_control(10, speeds[9], RIGHT_FOOT_DOWN[9])
  speed_pos_control(11, speeds[10], RIGHT_FOOT_DOWN[10])
  speed_pos_control(12, speeds[11], RIGHT_FOOT_DOWN[11])
  spinWhileMoving(RIGHT_FOOT_DOWN)
  speeds = calculateServoSpeed(RIGHT_FOOT_TORSO)
  speed_pos_control(1, speeds[0], RIGHT_FOOT_TORSO[0])
  speed_pos_control(2, speeds[1], RIGHT_FOOT_TORSO[1])
  speed_pos_control(3, speeds[2], RIGHT_FOOT_TORSO[2])
  speed_pos_control(4, speeds[3], RIGHT_FOOT_TORSO[3])
  speed_pos_control(5, speeds[4], RIGHT_FOOT_TORSO[4])
  speed_pos_control(6, speeds[5], RIGHT_FOOT_TORSO[5])
  speed_pos_control(7, speeds[6], RIGHT_FOOT_TORSO[6])
  speed_pos_control(8, speeds[7], RIGHT_FOOT_TORSO[7])
  speed_pos_control(9, speeds[8], RIGHT_FOOT_TORSO[8])
  speed_pos_control(10, speeds[9], RIGHT_FOOT_TORSO[9])
  speed_pos_control(11, speeds[10], RIGHT_FOOT_TORSO[10])
  speed_pos_control(12, speeds[11], RIGHT_FOOT_TORSO[11])
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
  speeds = calculateServoSpeed(RIGHT_LEG_STRAIGHT)
  speed_pos_control(1, speeds[0], RIGHT_LEG_STRAIGHT[0])
  speed_pos_control(2, speeds[1], RIGHT_LEG_STRAIGHT[1])
  speed_pos_control(3, speeds[2], RIGHT_LEG_STRAIGHT[2])
  speed_pos_control(4, speeds[3], RIGHT_LEG_STRAIGHT[3])
  speed_pos_control(5, speeds[4], RIGHT_LEG_STRAIGHT[4])
  speed_pos_control(6, speeds[5], RIGHT_LEG_STRAIGHT[5])
  speed_pos_control(7, speeds[6], RIGHT_LEG_STRAIGHT[6])
  speed_pos_control(8, speeds[7], RIGHT_LEG_STRAIGHT[7])
  speed_pos_control(9, speeds[8], RIGHT_LEG_STRAIGHT[8])
  speed_pos_control(10, speeds[9], RIGHT_LEG_STRAIGHT[9])
  speed_pos_control(11, speeds[10], RIGHT_LEG_STRAIGHT[10])
  speed_pos_control(12, speeds[11], RIGHT_LEG_STRAIGHT[11])
  spinWhileMoving(RIGHT_LEG_STRAIGHT)
  speeds = calculateServoSpeed(LEFT_FOOT_FORWARD)
  speed_pos_control(1, speeds[0], LEFT_FOOT_FORWARD[0])
  speed_pos_control(2, speeds[1], LEFT_FOOT_FORWARD[1])
  speed_pos_control(3, speeds[2], LEFT_FOOT_FORWARD[2])
  speed_pos_control(4, speeds[3], LEFT_FOOT_FORWARD[3])
  speed_pos_control(5, speeds[4], LEFT_FOOT_FORWARD[4])
  speed_pos_control(6, speeds[5], LEFT_FOOT_FORWARD[5])
  speed_pos_control(7, speeds[6], LEFT_FOOT_FORWARD[6])
  speed_pos_control(8, speeds[7], LEFT_FOOT_FORWARD[7])
  speed_pos_control(9, speeds[8], LEFT_FOOT_FORWARD[8])
  speed_pos_control(10, speeds[9], LEFT_FOOT_FORWARD[9])
  speed_pos_control(11, speeds[10], LEFT_FOOT_FORWARD[10])
  speed_pos_control(12, speeds[11], LEFT_FOOT_FORWARD[11])
  spinWhileMoving(LEFT_FOOT_FORWARD)
  speeds = calculateServoSpeed(LEFT_FOOT_DOWN)
  speed_pos_control(1, speeds[0], LEFT_FOOT_DOWN[0])
  speed_pos_control(2, speeds[1], LEFT_FOOT_DOWN[1])
  speed_pos_control(3, speeds[2], LEFT_FOOT_DOWN[2])
  speed_pos_control(4, speeds[3], LEFT_FOOT_DOWN[3])
  speed_pos_control(5, speeds[4], LEFT_FOOT_DOWN[4])
  speed_pos_control(6, speeds[5], LEFT_FOOT_DOWN[5])
  speed_pos_control(7, speeds[6], LEFT_FOOT_DOWN[6])
  speed_pos_control(8, speeds[7], LEFT_FOOT_DOWN[7])
  speed_pos_control(9, speeds[8], LEFT_FOOT_DOWN[8])
  speed_pos_control(10, speeds[9], LEFT_FOOT_DOWN[9])
  speed_pos_control(11, speeds[10], LEFT_FOOT_DOWN[10])
  speed_pos_control(12, speeds[11], LEFT_FOOT_DOWN[11])
  spinWhileMoving(LEFT_FOOT_DOWN)
  speeds = calculateServoSpeed(LEFT_FOOT_TORSO)
  speed_pos_control(1, speeds[0], LEFT_FOOT_TORSO[0])
  speed_pos_control(2, speeds[1], LEFT_FOOT_TORSO[1])
  speed_pos_control(3, speeds[2], LEFT_FOOT_TORSO[2])
  speed_pos_control(4, speeds[3], LEFT_FOOT_TORSO[3])
  speed_pos_control(5, speeds[4], LEFT_FOOT_TORSO[4])
  speed_pos_control(6, speeds[5], LEFT_FOOT_TORSO[5])
  speed_pos_control(7, speeds[6], LEFT_FOOT_TORSO[6])
  speed_pos_control(8, speeds[7], LEFT_FOOT_TORSO[7])
  speed_pos_control(9, speeds[8], LEFT_FOOT_TORSO[8])
  speed_pos_control(10, speeds[9], LEFT_FOOT_TORSO[9])
  speed_pos_control(11, speeds[10], LEFT_FOOT_TORSO[10])
  speed_pos_control(12, speeds[11], LEFT_FOOT_TORSO[11])
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
  speeds = calculateServoSpeed(LEFT_LEG_STRAIGHT)
  speed_pos_control(1, speeds[0], LEFT_LEG_STRAIGHT[0])
  speed_pos_control(2, speeds[1], LEFT_LEG_STRAIGHT[1])
  speed_pos_control(3, speeds[2], LEFT_LEG_STRAIGHT[2])
  speed_pos_control(4, speeds[3], LEFT_LEG_STRAIGHT[3])
  speed_pos_control(5, speeds[4], LEFT_LEG_STRAIGHT[4])
  speed_pos_control(6, speeds[5], LEFT_LEG_STRAIGHT[5])
  speed_pos_control(7, speeds[6], LEFT_LEG_STRAIGHT[6])
  speed_pos_control(8, speeds[7], LEFT_LEG_STRAIGHT[7])
  speed_pos_control(9, speeds[8], LEFT_LEG_STRAIGHT[8])
  speed_pos_control(10, speeds[9], LEFT_LEG_STRAIGHT[9])
  speed_pos_control(11, speeds[10], LEFT_LEG_STRAIGHT[10])
  speed_pos_control(12, speeds[11], LEFT_LEG_STRAIGHT[11])
  spinWhileMoving(LEFT_LEG_STRAIGHT)
  speeds = calculateServoSpeed(RIGHT_FOOT_FORWARD)
  speed_pos_control(1, speeds[0], RIGHT_FOOT_FORWARD[0])
  speed_pos_control(2, speeds[1], RIGHT_FOOT_FORWARD[1])
  speed_pos_control(3, speeds[2], RIGHT_FOOT_FORWARD[2])
  speed_pos_control(4, speeds[3], RIGHT_FOOT_FORWARD[3])
  speed_pos_control(5, speeds[4], RIGHT_FOOT_FORWARD[4])
  speed_pos_control(6, speeds[5], RIGHT_FOOT_FORWARD[5])
  speed_pos_control(7, speeds[6], RIGHT_FOOT_FORWARD[6])
  speed_pos_control(8, speeds[7], RIGHT_FOOT_FORWARD[7])
  speed_pos_control(9, speeds[8], RIGHT_FOOT_FORWARD[8])
  speed_pos_control(10, speeds[9], RIGHT_FOOT_FORWARD[9])
  speed_pos_control(11, speeds[10], RIGHT_FOOT_FORWARD[10])
  speed_pos_control(12, speeds[11], RIGHT_FOOT_FORWARD[11])
  spinWhileMoving(RIGHT_FOOT_FORWARD)
  speeds = calculateServoSpeed(RIGHT_FOOT_DOWN)
  speed_pos_control(1, speeds[0], RIGHT_FOOT_DOWN[0])
  speed_pos_control(2, speeds[1], RIGHT_FOOT_DOWN[1])
  speed_pos_control(3, speeds[2], RIGHT_FOOT_DOWN[2])
  speed_pos_control(4, speeds[3], RIGHT_FOOT_DOWN[3])
  speed_pos_control(5, speeds[4], RIGHT_FOOT_DOWN[4])
  speed_pos_control(6, speeds[5], RIGHT_FOOT_DOWN[5])
  speed_pos_control(7, speeds[6], RIGHT_FOOT_DOWN[6])
  speed_pos_control(8, speeds[7], RIGHT_FOOT_DOWN[7])
  speed_pos_control(9, speeds[8], RIGHT_FOOT_DOWN[8])
  speed_pos_control(10, speeds[9], RIGHT_FOOT_DOWN[9])
  speed_pos_control(11, speeds[10], RIGHT_FOOT_DOWN[10])
  speed_pos_control(12, speeds[11], RIGHT_FOOT_DOWN[11])
  spinWhileMoving(RIGHT_FOOT_DOWN)
  speeds = calculateServoSpeed(RIGHT_FOOT_TORSO)
  speed_pos_control(1, speeds[0], RIGHT_FOOT_TORSO[0])
  speed_pos_control(2, speeds[1], RIGHT_FOOT_TORSO[1])
  speed_pos_control(3, speeds[2], RIGHT_FOOT_TORSO[2])
  speed_pos_control(4, speeds[3], RIGHT_FOOT_TORSO[3])
  speed_pos_control(5, speeds[4], RIGHT_FOOT_TORSO[4])
  speed_pos_control(6, speeds[5], RIGHT_FOOT_TORSO[5])
  speed_pos_control(7, speeds[6], RIGHT_FOOT_TORSO[6])
  speed_pos_control(8, speeds[7], RIGHT_FOOT_TORSO[7])
  speed_pos_control(9, speeds[8], RIGHT_FOOT_TORSO[8])
  speed_pos_control(10, speeds[9], RIGHT_FOOT_TORSO[9])
  speed_pos_control(11, speeds[10], RIGHT_FOOT_TORSO[10])
  speed_pos_control(12, speeds[11], RIGHT_FOOT_TORSO[11])
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

def closeStepLForward():
  speeds = calculateServoSpeed(LEFT_LEG_STRAIGHT)
  speed_pos_control(1, speeds[0], LEFT_LEG_STRAIGHT[0])
  speed_pos_control(2, speeds[1], LEFT_LEG_STRAIGHT[1])
  speed_pos_control(3, speeds[2], LEFT_LEG_STRAIGHT[2])
  speed_pos_control(4, speeds[3], LEFT_LEG_STRAIGHT[3])
  speed_pos_control(5, speeds[4], LEFT_LEG_STRAIGHT[4])
  speed_pos_control(6, speeds[5], LEFT_LEG_STRAIGHT[5])
  speed_pos_control(7, speeds[6], LEFT_LEG_STRAIGHT[6])
  speed_pos_control(8, speeds[7], LEFT_LEG_STRAIGHT[7])
  speed_pos_control(9, speeds[8], LEFT_LEG_STRAIGHT[8])
  speed_pos_control(10, speeds[9], LEFT_LEG_STRAIGHT[9])
  speed_pos_control(11, speeds[10], LEFT_LEG_STRAIGHT[10])
  speed_pos_control(12, speeds[11], LEFT_LEG_STRAIGHT[11])
  speeds = calculateServoSpeed(STAND)
  speed_pos_control(1, speeds[0], STAND[0])
  speed_pos_control(2, speeds[1], STAND[1])
  speed_pos_control(3, speeds[2], STAND[2])
  speed_pos_control(4, speeds[3], STAND[3])
  speed_pos_control(5, speeds[4], STAND[4])
  speed_pos_control(6, speeds[5], STAND[5])
  speed_pos_control(7, speeds[6], STAND[6])
  speed_pos_control(8, speeds[7], STAND[7])
  speed_pos_control(9, speeds[8], STAND[8])
  speed_pos_control(10, speeds[9], STAND[9])
  speed_pos_control(11, speeds[10], STAND[10])
  speed_pos_control(12, speeds[11], STAND[11])
  spinWhileMoving(STAND)

def closeStepRForward():
  speeds = calculateServoSpeed(RIGHT_LEG_STRAIGHT)
  speed_pos_control(1, speeds[0], RIGHT_LEG_STRAIGHT[0])
  speed_pos_control(2, speeds[1], RIGHT_LEG_STRAIGHT[1])
  speed_pos_control(3, speeds[2], RIGHT_LEG_STRAIGHT[2])
  speed_pos_control(4, speeds[3], RIGHT_LEG_STRAIGHT[3])
  speed_pos_control(5, speeds[4], RIGHT_LEG_STRAIGHT[4])
  speed_pos_control(6, speeds[5], RIGHT_LEG_STRAIGHT[5])
  speed_pos_control(7, speeds[6], RIGHT_LEG_STRAIGHT[6])
  speed_pos_control(8, speeds[7], RIGHT_LEG_STRAIGHT[7])
  speed_pos_control(9, speeds[8], RIGHT_LEG_STRAIGHT[8])
  speed_pos_control(10, speeds[9], RIGHT_LEG_STRAIGHT[9])
  speed_pos_control(11, speeds[10], RIGHT_LEG_STRAIGHT[10])
  speed_pos_control(12, speeds[11], RIGHT_LEG_STRAIGHT[11])
  speeds = calculateServoSpeed(STAND)
  speed_pos_control(1, speeds[0], STAND[0])
  speed_pos_control(2, speeds[1], STAND[1])
  speed_pos_control(3, speeds[2], STAND[2])
  speed_pos_control(4, speeds[3], STAND[3])
  speed_pos_control(5, speeds[4], STAND[4])
  speed_pos_control(6, speeds[5], STAND[5])
  speed_pos_control(7, speeds[6], STAND[6])
  speed_pos_control(8, speeds[7], STAND[7])
  speed_pos_control(9, speeds[8], STAND[8])
  speed_pos_control(10, speeds[9], STAND[9])
  speed_pos_control(11, speeds[10], STAND[10])
  speed_pos_control(12, speeds[11], STAND[11])
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
  if forwardFoot == ForwardFoot.left:
    closeStepLForward()
  else:
    closeStepRForward()
  forwardFoot = ForwardFoot.neither

if __name__ == "__main__":
  walkLogic(5)
