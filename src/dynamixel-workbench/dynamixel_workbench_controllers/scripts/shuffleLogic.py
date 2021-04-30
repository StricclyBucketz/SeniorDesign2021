#!/usr/bin/env python3

import rospy
from enum import IntEnum
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

class PosSpeed(IntEnum):
  fullTwist = 0
  fourTen = 1
  twoEight = 2
  threeNineHalfTwist = 3
  fullFeetHips = 4
  halfFeetHips = 5

DELAY = 0.225
TOLERANCE = 10 # %tolerance = TOLERANCE / 1023 (In this case ~0.3%)
BASE_MOTOR_SPEED = 128 # Sets the speed for the servo moving the farthest in a state change.
STANDING = np.array([508,681,157,298,508,512,515,342,866,725,515,512,537,574,470,486,449,553])
LAnkleHip = np.array([508, 483, 533]) # Left ankle (1) and hip (5) default, rotated left, rotated right
RAnkleHip = np.array([515, 490, 540]) # Right ankle (7) and hip (11) default, rotated left, rotated right
TwoPos = np.array([681, 656]) # Right ankle pitch default and rotated up
FourPos = np.array([298, 273]) # Right hip pitch default and rotated up
EightPos = np.array([342, 367]) # Left ankle pitch default and rotated up
TenPos = np.array([725, 750]) # Left hip pitch default and rotated up

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

def Stand():
  for i in range(18):
    position_control(i+1, STANDING[i])

def stepOff():
  # Standing -> LHipStanding
  position_control(1, LAnkleHip[1])
  position_control(5, LAnkleHip[2])
  position_control(7, RAnkleHip[1])
  position_control(11, RAnkleHip[2])
  time.sleep(DELAY)

  # LHipStanding -> LHipRForward
  position_control(2, TwoPos[1])
  position_control(4, FourPos[1])
  time.sleep(DELAY)

def leftStep():
  # LHipRForward -> RHipRForward
  position_control(1, LAnkleHip[2])
  position_control(5, LAnkleHip[1])
  position_control(7, RAnkleHip[2])
  position_control(11, RAnkleHip[1])
  time.sleep(DELAY)

  # RHipRForward -> RHipStanding
  position_control(2, TwoPos[0]) #1
  position_control(4, FourPos[0])
  # position_control(8, EightPos[1]) #2.1
  # position_control(10, TenPos[1])
  # time.sleep(DELAY)
  # position_control(2, TwoPos[0]) #2.2
  # position_control(4, FourPos[0])
  # position_control(8, EightPos[0])
  # position_control(10, TenPos[0])
  time.sleep(DELAY)

  # RHipStanding -> RHipLForward
  position_control(8, EightPos[1])
  position_control(10, TenPos[1])
  time.sleep(DELAY)

def rightStep():
  # RHipLForward -> LHipLForward
  position_control(1, LAnkleHip[1])
  position_control(5, LAnkleHip[2])
  position_control(7, RAnkleHip[1])
  position_control(11, RAnkleHip[2])
  time.sleep(DELAY)

  # LHipLForward -> LHipStanding
  position_control(8, EightPos[0]) #1
  position_control(10, TenPos[0])
  # position_control(2, TwoPos[1]) #2.1
  # position_control(4, FourPos[1])
  # time.sleep(DELAY)
  # position_control(2, TwoPos[0]) #2.2
  # position_control(4, FourPos[0])
  # position_control(8, EightPos[0])
  # position_control(10, TenPos[0])
  time.sleep(DELAY)

  # LHipStanding -> LHipRForward
  position_control(2, TwoPos[1])
  position_control(4, FourPos[1])
  time.sleep(DELAY)

def closeStepLeftForward():
  # RHipLForward -> LHipLForward
  position_control(1, LAnkleHip[1])
  position_control(5, LAnkleHip[2])
  position_control(7, RAnkleHip[1])
  position_control(11, RAnkleHip[2])
  time.sleep(DELAY)

  # LHipLForward -> LHipStanding
  position_control(8, EightPos[0]) #1
  position_control(10, TenPos[0])
  # position_control(2, TwoPos[1]) #2.1
  # position_control(4, FourPos[1])
  # time.sleep(DELAY)
  # position_control(2, TwoPos[0]) #2.2
  # position_control(4, FourPos[0])
  # position_control(8, EightPos[0])
  # position_control(10, TenPos[0])
  time.sleep(DELAY)

  # LHipStanding -> Standing
  position_control(1, LAnkleHip[0])
  position_control(5, LAnkleHip[0])
  position_control(7, RAnkleHip[0])
  position_control(11, RAnkleHip[0])  
  time.sleep(DELAY)

def closeStepRightForward():
  # LHipRForward -> RHipRForward
  position_control(1, LAnkleHip[2])
  position_control(5, LAnkleHip[1])
  position_control(7, RAnkleHip[2])
  position_control(11, RAnkleHip[1])
  time.sleep(DELAY)

  # RHipRForward -> RHipStanding
  position_control(2, TwoPos[0]) #1
  position_control(4, FourPos[0])
  # position_control(8, EightPos[1]) #2.1
  # position_control(10, TenPos[1])
  # time.sleep(DELAY)
  # position_control(2, TwoPos[0]) #2.2
  # position_control(4, FourPos[0])
  # position_control(8, EightPos[0])
  # position_control(10, TenPos[0])
  time.sleep(DELAY)

  # RHipStanding -> Standing
  position_control(1, LAnkleHip[0])
  position_control(5, LAnkleHip[0])
  position_control(7, RAnkleHip[0])
  position_control(11, RAnkleHip[0]) 
  time.sleep(DELAY)

def walkLogic(stepsToTake):
  stepsTaken = 0
  nextIsRight = True
  # isFirstLast = True
  while stepsTaken < stepsToTake:
    if stepsTaken == 0:
      stepOff()
      nextIsRight = not nextIsRight
    else:
      if nextIsRight:
        rightStep()
        nextIsRight = not nextIsRight
      else:
        leftStep()
        nextIsRight = not nextIsRight
    stepsTaken += 1
  if nextIsRight:
    closeStepLeftForward()
  else:
    closeStepRightForward()

if __name__ == "__main__":

  Stand()
  time.sleep(DELAY)
  for i in range(18):
    speed_control(i + 1, BASE_MOTOR_SPEED)
  time.sleep(DELAY)
  walkLogic(5) # TODO make this user input
  # rightStep()
  # leftStep()
