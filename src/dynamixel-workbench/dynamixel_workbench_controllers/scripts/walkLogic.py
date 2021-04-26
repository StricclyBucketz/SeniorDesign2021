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

DELAY = 0.225
TOLERANCE = 10 # %tolerance = TOLERANCE / 1023 (In this case ~0.3%)
BASE_MOTOR_SPEED = 256 # Sets the speed for the servo moving the farthest in a state change.
STATIC = np.array([512,312,712])    # Default for not {2,3,4,8,9,10}, constant for 3, constant for 9
TWO_POS = np.array([622,572])       # Default for servo 2, its extended state
FOUR_POS = np.array([412,362])      # Default for servo 4, its extended state
EIGHT_POS = np.array([402,452])     # Default for servo 8, its extended state
TEN_POS = np.array([612,662])       # Defulat for servo 10, its extended state
FEETHIPS = np.array([487,537])      # When not 512, both hips are one, both feet are other
SPEED = np.array([BASE_MOTOR_SPEED, math.ceil(BASE_MOTOR_SPEED / 2)]) # IF first/last step: {2,4,8,10},{1,5,7,11} ELSE: {all}, {}

targets = np.zeros(6)

indices = np.zeros(6)

# class speed_calc:

#   def __init__(self):
#     self.deltas = np.zeros(18)
#     self.sub = rospy.Subscriber("dynamixel_workbench/dynamixel_state", msg.DynamixelStateList, self.callback)

#   def callback(self, data):
#       # loop through each servo, recording (in array) difference between each current position and target position (delta)
#     # find greatestDelta
#     deltas = targets
#     array = np.zeros(18)
#     for servoInfo in data.dynamixel_state:
#       array[servoInfo.id-1] = servoInfo.present_position

#     maxDelta = -1024
#     i = 0
#     for delta in deltas:
#       delta = abs(delta - array[i])
#       if delta > maxDelta:
#         maxDelta = delta
#       i = i + 1
#     for delta in deltas:
#       delta = math.ceil(BASE_MOTOR_SPEED * delta/maxDelta)

class imu_read:

  def __init__(self):
    self.roll = 0
    self.pitch = 0
    self.yaw = 0
    rospy.init_node('imu_Node', anonymous=True)
    sub = rospy.Subscriber("navx_micro/euler", Vector3Stamped, imu_printer)

  def callback(self, data):
      # loop through each servo, recording (in array) difference between each current position and target position (delta)
    # find greatestDelta
    self.pitch = data.vector.x
    self.roll = data.vector.y
    self.yaw = data.vector.z

def motor_callback(motorSub):

  #rate = rospy.Rate(10)
  moving = True
  while moving:
    i = 0
    for index in indices:
      servoInfo = motorSub.dynamixel_state[int(index)]
      moving = moving and abs(servoInfo.present_position - targets[i]) > TOLERANCE
      i += 1

# def calculateServoSpeed():

#   deltas = speed_calc()
#   return deltas.deltas


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

def leftStep():
  indices = np.array([1,5,7,11,8,10])

  targets = np.array([FEETHIPS[0],FEETHIPS[1],FEETHIPS[0],FEETHIPS[1],EIGHT_POS[1],TEN_POS[1]])
  position_control(1, FEETHIPS[0])
  position_control(5, FEETHIPS[1])
  position_control(7, FEETHIPS[0])
  position_control(11, FEETHIPS[1])
  position_control(2, EIGHT_POS[1])
  position_control(4, TEN_POS[1])
  rospy.loginfo("Entering spin lock")
  #spinWhileMoving()
  time.sleep(DELAY)

  targets = np.array([FEETHIPS[1],FEETHIPS[0],FEETHIPS[1],FEETHIPS[0],EIGHT_POS[1],TEN_POS[1]])
  position_control(1, FEETHIPS[1])
  position_control(5, FEETHIPS[0])
  position_control(7, FEETHIPS[1])
  position_control(11, FEETHIPS[0])
  position_control(2, EIGHT_POS[0])
  position_control(4, TEN_POS[0])
  #spinWhileMoving()
  time.sleep(DELAY)


def rightStep():
  indices = np.array([1,5,7,11,2,4])

  targets = np.array([FEETHIPS[1],FEETHIPS[0],FEETHIPS[1],FEETHIPS[0],TWO_POS[1],FOUR_POS[1]])
  position_control(1, FEETHIPS[1])
  position_control(5, FEETHIPS[0])
  position_control(7, FEETHIPS[1])
  position_control(11, FEETHIPS[0])
  position_control(2, TWO_POS[1])
  position_control(4, FOUR_POS[1])
  #spinWhileMoving()
  time.sleep(DELAY)

  targets = np.array([FEETHIPS[0],FEETHIPS[1],FEETHIPS[0],FEETHIPS[1],TWO_POS[0],FOUR_POS[0]])
  position_control(1, FEETHIPS[0])
  position_control(5, FEETHIPS[1])
  position_control(7, FEETHIPS[0])
  position_control(11, FEETHIPS[1])
  position_control(2, TWO_POS[0])
  position_control(4, FOUR_POS[0])
  #spinWhileMoving()
  time.sleep(DELAY)

def walkLogic(stepsToTake):
  stepsTaken = 0
  nextIsRight = True
  # isFirstLast = True
  while stepsTaken < stepsToTake:
    if stepsTaken == 1:
      # isFirstLast = False
      speed_control(1, SPEED[0])
      speed_control(5, SPEED[0])
      speed_control(7, SPEED[0])
      speed_control(11, SPEED[0])
    if stepsTaken == 0:
      # isFirstLast = True
      speed_control(1, SPEED[1])
      speed_control(5, SPEED[1])
      speed_control(7, SPEED[1])
      speed_control(11, SPEED[1])
      speed_control(2, SPEED[0])
      speed_control(4, SPEED[0])
      speed_control(8, SPEED[0])
      speed_control(10, SPEED[0])
    elif stepsTaken == stepsToTake - 1:
      # isFirstLast = True
      speed_control(1, SPEED[1])
      speed_control(5, SPEED[1])
      speed_control(7, SPEED[1])
      speed_control(11, SPEED[1])
    if nextIsRight:
      rightStep()
      nextIsRight = not nextIsRight
    else:
      leftStep()
      nextIsRight = not nextIsRight
    stepsTaken += 1
  position_control(1, STATIC[0])
  position_control(5, STATIC[0])
  position_control(6, STATIC[0])
  position_control(11, STATIC[0])

if __name__ == "__main__":

  position_control(1, STATIC[0])
  position_control(2, TWO_POS[0])
  position_control(3, STATIC[1])
  position_control(4, FOUR_POS[1])
  position_control(5, STATIC[0])
  position_control(6, STATIC[0])
  position_control(7, STATIC[0])
  position_control(8, EIGHT_POS[0])
  position_control(9, STATIC[2])
  position_control(10, TEN_POS[1])
  position_control(11, STATIC[0])
  position_control(12, STATIC[0])
  time.sleep(DELAY)
  walkLogic(3) # TODO make this user input
