#!/usr/bin/env python3

import rospy
import enum
import sys
import numpy as np
import math
import time

from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs import msg
from dynamixel_workbench_msgs import srv

import navx_micro
from navx_micro._impl.imuregisters import IMURegisters
from navx_micro._impl.ahrsprotocol import AHRSProtocol
from geometry_msgs.msg import Quaternion, Vector3, Vector3Stamped
from sensor_msgs.msg import Imu

from functions import speed_control, position_control

DELAY = 0.1
TOLERANCE = 10 # %tolerance = TOLERANCE / 1023 (In this case ~0.3%)
BASE_MOTOR_SPEED = 192 # Sets the speed for the servo moving the farthest in a state change.
STAND = np.array([512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512])
RIGHT_FOOT_FORWARD = np.array([512,512,720,750,512,512,512,512,512,512,512,512,512,512,512,512,512,512])
RIGHT_FOOT_DOWN = np.array([512,512,720,750,512,512,512,405,455,565,512,512,512,512,512,512,512,512])
RIGHT_FOOT_TORSO = np.array([512,512,720,740,512,512,512,405,455,565,512,512,512,512,512,512,512,512])
RIGHT_LEG_STRAIGHT = np.array([512,512,512,512,512,512,512,405,455,565,512,512,512,512,512,512,512,512])
LEFT_FOOT_FORWARD = np.array([512,512,512,512,512,512,512,512,304,295,512,512,512,512,512,512,512,512])
LEFT_FOOT_DOWN = np.array([512,620,570,464,512,512,512,512,304,295,512,512,512,512,512,512,512,512])
LEFT_FOOT_TORSO = np.array([512,620,570,464,512,512,512,512,304,299,512,512,512,512,512,512,512,512])
LEFT_LEG_STRAIGHT = np.array([512,620,570,464,512,512,512,512,512,512,512,512,512,512,512,512,512,512])

targets = np.zeros(18)

PREVIOUS_STATE = np.zeros(18)

class speed_calc:

  def __init__(self):
    self.deltas = np.zeros(18)
    self.sub = rospy.Subscriber("dynamixel_workbench/dynamixel_state", msg.DynamixelStateList, self.callback)

  def callback(self, data):
      # loop through each servo, recording (in array) difference between each current position and target position (delta)
    # find greatestDelta
    deltas = targets
    array = np.zeros(18)
    for servoInfo in data.dynamixel_state:
      array[servoInfo.id-1] = servoInfo.present_position

    maxDelta = -1024
    i = 0
    for delta in deltas:
      delta = abs(delta - array[i])
      if delta > maxDelta:
        maxDelta = delta
      i = i + 1
    for delta in deltas:
      delta = math.ceil(BASE_MOTOR_SPEED * delta/maxDelta)


class ForwardFoot(enum.Enum):
  neither = 0
  right = 1
  left = 2

def motor_callback(motorSub):

  rate = rospy.Rate(10)
  moving = True
  while moving:
    for servoInfo in motorSub.dynamixel_state:
      moving = moving and abs(servoInfo.present_position - targets[servoInfo.id-1]) > TOLERANCE


def calculateServoSpeed():

  deltas = speed_calc()
  return deltas.deltas


def servoInfoIndexTest():
  # servoInfos read in via ROS
  rospy.init_node('motorSub', anonymous=True)
  motorSub = rospy.Subscriber("dynamixel_workbench/dynamixel_state", msg.DynamixelStateList, callback)
  for dynamixel_state in motorSub:
    print("Servo {} is at position: {}".format(motorSub.dynamixel_state.id, motorSub.dynamixel_state.present_position))


# Takes a numpy.array of size 12.  Spins until position read in from servos are all within tolerance
def spinWhileMoving():

  rospy.init_node('motorSub', anonymous=True)
  posSub = rospy.Subscriber("dynamixel_workbench/dynamixel_state", msg.DynamixelStateList, motor_callback)

  #TODO find the indeces of the motors in DynamixelStateList and populate below
  #Christian will do this when we can test


def stepOff():
  # TODO Also determine if we can get away with only calling the motors that need to move for each step instead of all of them
  # TODO test and see if we need to implement time.sleep() in between certain lines
  #rospy.loginfo("Step Off: ")
  #rospy.loginfo("RIGHT_FOOT_FORWARD")
  targets = RIGHT_FOOT_FORWARD
  # speeds = calculateServoSpeed()
  #speed_pos_control(1, speeds[0], RIGHT_FOOT_FORWARD[0])
  #speed_pos_control(2, speeds[1], RIGHT_FOOT_FORWARD[1])
  speed_control(3, math.ceil(0.874*BASE_MOTOR_SPEED))
  speed_control(4, BASE_MOTOR_SPEED)
  position_control(3, RIGHT_FOOT_FORWARD[2])
  position_control(4, RIGHT_FOOT_FORWARD[3])
  time.sleep(DELAY)
  #speed_pos_control(5, speeds[4], RIGHT_FOOT_FORWARD[4])
  #speed_pos_control(6, speeds[5], RIGHT_FOOT_FORWARD[5])
  #speed_pos_control(7, speeds[6], RIGHT_FOOT_FORWARD[6])
  # speed_pos_control(8, speeds[7], RIGHT_FOOT_FORWARD[7])
  # speed_pos_control(9, speeds[8], RIGHT_FOOT_FORWARD[8])
  # speed_pos_control(10, speeds[9], RIGHT_FOOT_FORWARD[9])
  # speed_pos_control(11, speeds[10], RIGHT_FOOT_FORWARD[10])
  # speed_pos_control(12, speeds[11], RIGHT_FOOT_FORWARD[11])
  #spinWhileMoving()

  #rospy.loginfo("RIGHT_FOOT_DOWN")
  targets = RIGHT_FOOT_DOWN
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], RIGHT_FOOT_DOWN[0])
  # speed_pos_control(2, speeds[1], RIGHT_FOOT_DOWN[1])
  # speed_pos_control(3, speeds[2], RIGHT_FOOT_DOWN[2])
  # speed_pos_control(4, speeds[3], RIGHT_FOOT_DOWN[3])
  # speed_pos_control(5, speeds[4], RIGHT_FOOT_DOWN[4])
  # speed_pos_control(6, speeds[5], RIGHT_FOOT_DOWN[5])
  # speed_pos_control(7, speeds[6], RIGHT_FOOT_DOWN[6])
  speed_control(8, BASE_MOTOR_SPEED)
  speed_control(9, math.ceil(0.533*BASE_MOTOR_SPEED))
  speed_control(10, math.ceil(0.495*BASE_MOTOR_SPEED))
  position_control(8, RIGHT_FOOT_DOWN[7])
  position_control(9, RIGHT_FOOT_DOWN[8])
  position_control(10, RIGHT_FOOT_DOWN[9])
  time.sleep(DELAY)
  # speed_pos_control(11, speeds[10], RIGHT_FOOT_DOWN[10])
  # speed_pos_control(12, speeds[11], RIGHT_FOOT_DOWN[11])
  #spinWhileMoving()

  #rospy.loginfo("RIGHT_FOOT_TORSO")
  # targets = RIGHT_FOOT_TORSO
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], RIGHT_FOOT_TORSO[0])
  # speed_pos_control(2, speeds[1], RIGHT_FOOT_TORSO[1])
  # speed_pos_control(3, speeds[2], RIGHT_FOOT_TORSO[2])
  # speed_pos_control(4, speeds[3], RIGHT_FOOT_TORSO[3])
  # speed_pos_control(5, speeds[4], RIGHT_FOOT_TORSO[4])
  # speed_pos_control(6, speeds[5], RIGHT_FOOT_TORSO[5])
  # speed_pos_control(7, speeds[6], RIGHT_FOOT_TORSO[6])
  # speed_pos_control(8, speeds[7], RIGHT_FOOT_TORSO[7])
  # speed_pos_control(9, speeds[8], RIGHT_FOOT_TORSO[8])
  # speed_pos_control(10, speeds[9], RIGHT_FOOT_TORSO[9])
  # speed_pos_control(11, speeds[10], RIGHT_FOOT_TORSO[10])
  # speed_pos_control(12, speeds[11], RIGHT_FOOT_TORSO[11])
  # spinWhileMoving() # could be replaced by righting IMU
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
  #rospy.loginfo("Left Step: ")
  #rospy.loginfo("RIGHT_LEG_STRAIGHT")
  targets = RIGHT_LEG_STRAIGHT
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], RIGHT_LEG_STRAIGHT[0])
  # speed_pos_control(2, speeds[1], RIGHT_LEG_STRAIGHT[1])
  speed_control(3, math.ceil(0.874*BASE_MOTOR_SPEED))
  speed_control(4, BASE_MOTOR_SPEED)
  position_control(3, RIGHT_LEG_STRAIGHT[2])
  position_control(4, RIGHT_LEG_STRAIGHT[3])
  time.sleep(DELAY)
  # speed_pos_control(5, speeds[4], RIGHT_LEG_STRAIGHT[4])
  # speed_pos_control(6, speeds[5], RIGHT_LEG_STRAIGHT[5])
  # speed_pos_control(7, speeds[6], RIGHT_LEG_STRAIGHT[6])
  # speed_pos_control(8, speeds[7], RIGHT_LEG_STRAIGHT[7])
  # speed_pos_control(9, speeds[8], RIGHT_LEG_STRAIGHT[8])
  # speed_pos_control(10, speeds[9], RIGHT_LEG_STRAIGHT[9])
  # speed_pos_control(11, speeds[10], RIGHT_LEG_STRAIGHT[10])
  # speed_pos_control(12, speeds[11], RIGHT_LEG_STRAIGHT[11])
  #spinWhileMoving()

  #rospy.loginfo("LEFT_FOOT_FORWARD")
  targets = LEFT_FOOT_FORWARD
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], LEFT_FOOT_FORWARD[0])
  # speed_pos_control(2, speeds[1], LEFT_FOOT_FORWARD[1])
  # speed_pos_control(3, speeds[2], LEFT_FOOT_FORWARD[2])
  # speed_pos_control(4, speeds[3], LEFT_FOOT_FORWARD[3])
  # speed_pos_control(5, speeds[4], LEFT_FOOT_FORWARD[4])
  # speed_pos_control(6, speeds[5], LEFT_FOOT_FORWARD[5])
  # speed_pos_control(7, speeds[6], LEFT_FOOT_FORWARD[6])
  # speed_pos_control(8, speeds[7], LEFT_FOOT_FORWARD[7])
  speed_control(9, math.ceil(0.382*BASE_MOTOR_SPEED))
  speed_control(10, math.ceil(0.539*BASE_MOTOR_SPEED))
  speed_control(11, BASE_MOTOR_SPEED)
  position_control(9, LEFT_FOOT_FORWARD[8])
  position_control(10, LEFT_FOOT_FORWARD[9])
  position_control(11, LEFT_FOOT_FORWARD[10])
  time.sleep(DELAY)
  # speed_pos_control(12, speeds[11], LEFT_FOOT_FORWARD[11])
  #spinWhileMoving()

  #rospy.loginfo("LEFT_FOOT_DOWN")
  targets = LEFT_FOOT_DOWN
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], LEFT_FOOT_DOWN[0])
  speed_control(2, BASE_MOTOR_SPEED)
  speed_control(3, math.ceil(0.537*BASE_MOTOR_SPEED))
  speed_control(4, math.ceil(0.491*BASE_MOTOR_SPEED))
  position_control(2, LEFT_FOOT_DOWN[1])
  position_control(3, LEFT_FOOT_DOWN[2])
  position_control(4, LEFT_FOOT_DOWN[3])
  time.sleep(DELAY)
  # speed_pos_control(5, speeds[4], LEFT_FOOT_DOWN[4])
  # speed_pos_control(6, speeds[5], LEFT_FOOT_DOWN[5])
  # speed_pos_control(7, speeds[6], LEFT_FOOT_DOWN[6])
  # speed_pos_control(8, speeds[7], LEFT_FOOT_DOWN[7])
  # speed_pos_control(9, speeds[8], LEFT_FOOT_DOWN[8])
  # speed_pos_control(10, speeds[9], LEFT_FOOT_DOWN[9])
  # speed_pos_control(11, speeds[10], LEFT_FOOT_DOWN[10])
  # speed_pos_control(12, speeds[11], LEFT_FOOT_DOWN[11])
  #spinWhileMoving()

  #rospy.loginfo("LEFT_FOOT_TORSO")
  # targets = LEFT_FOOT_TORSO
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], LEFT_FOOT_TORSO[0])
  # speed_pos_control(2, speeds[1], LEFT_FOOT_TORSO[1])
  # speed_pos_control(3, speeds[2], LEFT_FOOT_TORSO[2])
  # speed_pos_control(4, speeds[3], LEFT_FOOT_TORSO[3])
  # speed_pos_control(5, speeds[4], LEFT_FOOT_TORSO[4])
  # speed_pos_control(6, speeds[5], LEFT_FOOT_TORSO[5])
  # speed_pos_control(7, speeds[6], LEFT_FOOT_TORSO[6])
  # speed_pos_control(8, speeds[7], LEFT_FOOT_TORSO[7])
  # speed_pos_control(9, speeds[8], LEFT_FOOT_TORSO[8])
  # speed_pos_control(10, speeds[9], LEFT_FOOT_TORSO[9])
  # speed_pos_control(11, speeds[10], LEFT_FOOT_TORSO[10])
  # speed_pos_control(12, speeds[11], LEFT_FOOT_TORSO[11])
  # spinWhileMoving() # could be replaced by righting IMU
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
#  rospy.loginfo("Right Step: ")
#  rospy.loginfo("LEFT_LEG_STRAIGHT")
  targets = LEFT_LEG_STRAIGHT
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], LEFT_LEG_STRAIGHT[0])
  # speed_pos_control(2, speeds[1], LEFT_LEG_STRAIGHT[1])
  # speed_pos_control(3, speeds[2], LEFT_LEG_STRAIGHT[2])
  # speed_pos_control(4, speeds[3], LEFT_LEG_STRAIGHT[3])
  # speed_pos_control(5, speeds[4], LEFT_LEG_STRAIGHT[4])
  # speed_pos_control(6, speeds[5], LEFT_LEG_STRAIGHT[5])
  # speed_pos_control(7, speeds[6], LEFT_LEG_STRAIGHT[6])
  # speed_pos_control(8, speeds[7], LEFT_LEG_STRAIGHT[7])
  speed_control(9, math.ceil(0.959*BASE_MOTOR_SPEED))
  speed_control(10, BASE_MOTOR_SPEED)
  position_control(9, LEFT_LEG_STRAIGHT[8])
  position_control(10, LEFT_LEG_STRAIGHT[9])
  time.sleep(DELAY)
  # speed_pos_control(11, speeds[10], LEFT_LEG_STRAIGHT[10])
  # speed_pos_control(12, speeds[11], LEFT_LEG_STRAIGHT[11])
  #spinWhileMoving()

#  rospy.loginfo("RIGHT_FOOT_FORWARD")
  targets = RIGHT_FOOT_FORWARD
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], RIGHT_FOOT_FORWARD[0])
  speed_control(2, math.ceil(0.365*BASE_MOTOR_SPEED))
  speed_control(3, math.ceil(0.507*BASE_MOTOR_SPEED))
  speed_control(4, BASE_MOTOR_SPEED)
  position_control(2, RIGHT_FOOT_FORWARD[1])
  position_control(3, RIGHT_FOOT_FORWARD[2])
  position_control(4, RIGHT_FOOT_FORWARD[3])
  time.sleep(DELAY)
  # speed_pos_control(5, speeds[4], RIGHT_FOOT_FORWARD[4])
  # speed_pos_control(6, speeds[5], RIGHT_FOOT_FORWARD[5])
  # speed_pos_control(7, speeds[6], RIGHT_FOOT_FORWARD[6])
  # speed_pos_control(8, speeds[7], RIGHT_FOOT_FORWARD[7])
  # speed_pos_control(9, speeds[8], RIGHT_FOOT_FORWARD[8])
  # speed_pos_control(10, speeds[9], RIGHT_FOOT_FORWARD[9])
  # speed_pos_control(11, speeds[10], RIGHT_FOOT_FORWARD[10])
  # speed_pos_control(12, speeds[11], RIGHT_FOOT_FORWARD[11])
  #spinWhileMoving()

#  rospy.loginfo("RIGHT_FOOT_DOWN")
  targets = RIGHT_FOOT_DOWN
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], RIGHT_FOOT_DOWN[0])
  # speed_pos_control(2, speeds[1], RIGHT_FOOT_DOWN[1])
  # speed_pos_control(3, speeds[2], RIGHT_FOOT_DOWN[2])
  # speed_pos_control(4, speeds[3], RIGHT_FOOT_DOWN[3])
  # speed_pos_control(5, speeds[4], RIGHT_FOOT_DOWN[4])
  # speed_pos_control(6, speeds[5], RIGHT_FOOT_DOWN[5])
  # speed_pos_control(7, speeds[6], RIGHT_FOOT_DOWN[6])
  speed_control(8, BASE_MOTOR_SPEED)
  speed_control(9, math.ceil(0.533*BASE_MOTOR_SPEED))
  speed_control(10, math.ceil(0.495*BASE_MOTOR_SPEED))
  position_control(8, RIGHT_FOOT_DOWN[7])
  position_control(9, RIGHT_FOOT_DOWN[8])
  position_control(10, RIGHT_FOOT_DOWN[9])
  time.sleep(DELAY)
  # speed_pos_control(11, speeds[10], RIGHT_FOOT_DOWN[10])
  # speed_pos_control(12, speeds[11], RIGHT_FOOT_DOWN[11])
  #spinWhileMoving()

#  rospy.loginfo("RIGHT_FOOT_TORSO")
  # targets = RIGHT_FOOT_TORSO
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], RIGHT_FOOT_TORSO[0])
  # speed_pos_control(2, speeds[1], RIGHT_FOOT_TORSO[1])
  # speed_pos_control(3, speeds[2], RIGHT_FOOT_TORSO[2])
  # speed_pos_control(4, speeds[3], RIGHT_FOOT_TORSO[3])
  # speed_pos_control(5, speeds[4], RIGHT_FOOT_TORSO[4])
  # speed_pos_control(6, speeds[5], RIGHT_FOOT_TORSO[5])
  # speed_pos_control(7, speeds[6], RIGHT_FOOT_TORSO[6])
  # speed_pos_control(8, speeds[7], RIGHT_FOOT_TORSO[7])
  # speed_pos_control(9, speeds[8], RIGHT_FOOT_TORSO[8])
  # speed_pos_control(10, speeds[9], RIGHT_FOOT_TORSO[9])
  # speed_pos_control(11, speeds[10], RIGHT_FOOT_TORSO[10])
  # speed_pos_control(12, speeds[11], RIGHT_FOOT_TORSO[11])
  # spinWhileMoving() # could be replaced by righting IMU
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
#  rospy.loginfo("Close Step Left Forward")
#  rospy.loginfo("LEFT_LEG_STRAIGHT")
  targets = LEFT_LEG_STRAIGHT
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], LEFT_LEG_STRAIGHT[0])
  # speed_pos_control(2, speeds[1], LEFT_LEG_STRAIGHT[1])
  # speed_pos_control(3, speeds[2], LEFT_LEG_STRAIGHT[2])
  # speed_pos_control(4, speeds[3], LEFT_LEG_STRAIGHT[3])
  # speed_pos_control(5, speeds[4], LEFT_LEG_STRAIGHT[4])
  # speed_pos_control(6, speeds[5], LEFT_LEG_STRAIGHT[5])
  # speed_pos_control(7, speeds[6], LEFT_LEG_STRAIGHT[6])
  # speed_pos_control(8, speeds[7], LEFT_LEG_STRAIGHT[7])
  speed_control(9, math.ceil(0.959*BASE_MOTOR_SPEED))
  speed_control(10, BASE_MOTOR_SPEED)
  position_control(9, LEFT_LEG_STRAIGHT[8])
  position_control(10, LEFT_LEG_STRAIGHT[9])
  time.sleep(DELAY)
  # speed_pos_control(11, speeds[10], LEFT_LEG_STRAIGHT[10])
  # speed_pos_control(12, speeds[11], LEFT_LEG_STRAIGHT[11])
  #spinWhileMoving()

#  rospy.loginfo("STAND")
  targets = STAND
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], STAND[0])
  speed_control(2, BASE_MOTOR_SPEED)
  speed_control(3, math.ceil(0.537*BASE_MOTOR_SPEED))
  speed_control(4, math.ceil(0.491*BASE_MOTOR_SPEED))
  position_control(2, STAND[1])
  position_control(3, STAND[2])
  position_control(4, STAND[3])
  time.sleep(DELAY)
  # speed_pos_control(5, speeds[4], STAND[4])
  # speed_pos_control(6, speeds[5], STAND[5])
  # speed_pos_control(7, speeds[6], STAND[6])
  # speed_pos_control(8, speeds[7], STAND[7])
  # speed_pos_control(9, speeds[8], STAND[8])
  # speed_pos_control(10, speeds[9], STAND[9])
  # speed_pos_control(11, speeds[10], STAND[10])
  # speed_pos_control(12, speeds[11], STAND[11])
  #spinWhileMoving()

def closeStepRForward():
#  rospy.loginfo("Close Step Right Forward")
#  rospy.loginfo("RIGHT_LEG_STRAIGHT")
  targets = RIGHT_LEG_STRAIGHT
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], RIGHT_LEG_STRAIGHT[0])
  # speed_pos_control(2, speeds[1], RIGHT_LEG_STRAIGHT[1])
  speed_control(3, math.ceil(0.874*BASE_MOTOR_SPEED))
  speed_control(4, BASE_MOTOR_SPEED)
  position_control(3, RIGHT_LEG_STRAIGHT[2])
  position_control(4, RIGHT_LEG_STRAIGHT[3])
  time.sleep(DELAY)
  # speed_pos_control(5, speeds[4], RIGHT_LEG_STRAIGHT[4])
  # speed_pos_control(6, speeds[5], RIGHT_LEG_STRAIGHT[5])
  # speed_pos_control(7, speeds[6], RIGHT_LEG_STRAIGHT[6])
  # speed_pos_control(8, speeds[7], RIGHT_LEG_STRAIGHT[7])
  # speed_pos_control(9, speeds[8], RIGHT_LEG_STRAIGHT[8])
  # speed_pos_control(10, speeds[9], RIGHT_LEG_STRAIGHT[9])
  # speed_pos_control(11, speeds[10], RIGHT_LEG_STRAIGHT[10])
  # speed_pos_control(12, speeds[11], RIGHT_LEG_STRAIGHT[11])

#  rospy.loginfo("STAND")
  targets = STAND
  # speeds = calculateServoSpeed()
  # speed_pos_control(1, speeds[0], STAND[0])
  # speed_pos_control(2, speeds[1], STAND[1])
  # speed_pos_control(3, speeds[2], STAND[2])
  # speed_pos_control(4, speeds[3], STAND[3])
  # speed_pos_control(5, speeds[4], STAND[4])
  # speed_pos_control(6, speeds[5], STAND[5])
  # speed_pos_control(7, speeds[6], STAND[6])
  speed_control(8, BASE_MOTOR_SPEED)
  speed_control(9, math.ceil(0.533*BASE_MOTOR_SPEED))
  speed_control(10, math.ceil(0.495*BASE_MOTOR_SPEED))
  position_control(8, STAND[7])
  position_control(9, STAND[8])
  position_control(10, STAND[9])
  time.sleep(DELAY)
  # speed_pos_control(11, speeds[10], STAND[10])
  # speed_pos_control(12, speeds[11], STAND[11])
  #spinWhileMoving()

def walkLogic(stepsToTake):
  closeStepLForward()
  stepsTaken = 0
  forwardFoot = ForwardFoot.neither
  while stepsTaken < stepsToTake:
    if forwardFoot == ForwardFoot.neither:
#      print(f"StepOff\tstep#: {(stepsTaken+1)}")
      stepOff()
      forwardFoot = ForwardFoot.right
    elif forwardFoot == ForwardFoot.right:
#      print(f"LeftStep\tstep#: {(stepsTaken+1)}")
      leftStep()
      forwardFoot = ForwardFoot.left
    elif forwardFoot == ForwardFoot.left:
#      print(f"RightStep\tstep#: {(stepsTaken+1)}")
      rightStep()
      forwardFoot = ForwardFoot.right
    else:
      sys.exit("ERROR: Invalid foot forward")
    stepsTaken += 1
 # print("CloseStep")
  if forwardFoot == ForwardFoot.left:
    closeStepLForward()
  else:
    closeStepRForward()
  forwardFoot = ForwardFoot.neither

if __name__ == "__main__":
  walkLogic(1)
