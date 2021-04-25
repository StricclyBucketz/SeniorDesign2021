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
FIVETWELVE = np.array([512,512,512])
R_NEUTRAL = np.array([612,312,412])
L_NEUTRAL = np.array([412,712,612])
R_FORWARD = np.array([512,412,362])
L_DOWN = np.array([387,712,637])
R_DOWN = np.array([637,312,387])
L_FORWARD = np.array([512,612,662])
R_SPEED = np.array([BASE_MOTOR_SPEED, BASE_MOTOR_SPEED, math.ceil(BASE_MOTOR_SPEED / 2), math.ceil(BASE_MOTOR_SPEED / 4), BASE_MOTOR_SPEED, math.ceil(BASE_MOTOR_SPEED / 4)])
L_SPEED = np.array([math.ceil(BASE_MOTOR_SPEED / 4), BASE_MOTOR_SPEED, math.ceil(BASE_MOTOR_SPEED / 4), BASE_MOTOR_SPEED, BASE_MOTOR_SPEED, math.ceil(BASE_MOTOR_SPEED / 2)])

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

def motor_callback(motorSub):

  rate = rospy.Rate(10)
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
  indices = np.array([2,3,4,8,9,10])
  speed_control(2, L_SPEED[0])
  speed_control(3, L_SPEED[1])
  speed_control(4, L_SPEED[2])
  speed_control(8, L_SPEED[3])
  speed_control(9, L_SPEED[4])
  speed_control(10, L_SPEED[5])

  targets = np.concatenate((R_DOWN, L_FORWARD), axis = None)
  position_control(2, R_DOWN[0])
  position_control(3, R_DOWN[1])
  position_control(4, R_DOWN[2])
  position_control(8, L_FORWARD[0])
  position_control(9, L_FORWARD[1])
  position_control(10, L_FORWARD[2])
  time.sleep(DELAY)
  # spinWhileMoving()

  targets = np.concatenate((R_NEUTRAL, L_FORWARD), axis = None)
  position_control(2, R_NEUTRAL[0])
  position_control(3, R_NEUTRAL[1])
  position_control(4, R_NEUTRAL[2])
  time.sleep(DELAY)
  # spinWhileMoving()

  targets = np.concatenate((R_NEUTRAL, L_NEUTRAL), axis = None)
  position_control(8, L_NEUTRAL[0])
  position_control(9, L_NEUTRAL[1])
  position_control(10, L_NEUTRAL[2])
  time.sleep(DELAY)
  # spinWhileMoving()


def rightStep():
  indices = np.array([2,3,4,8,9,10])
  speed_control(2, R_SPEED[0])
  speed_control(3, R_SPEED[1])
  speed_control(4, R_SPEED[2])
  speed_control(8, R_SPEED[3])
  speed_control(9, R_SPEED[4])
  speed_control(10, R_SPEED[5])

  targets = np.concatenate((R_FORWARD, L_DOWN), axis = None)
  position_control(2, R_FORWARD[0])
  position_control(3, R_FORWARD[1])
  position_control(4, R_FORWARD[2])
  position_control(8, L_DOWN[0])
  position_control(9, L_DOWN[1])
  position_control(10, L_DOWN[2])
  time.sleep(DELAY)
  # spinWhileMoving()

  targets = np.concatenate((R_FORWARD, L_NEUTRAL), axis = None)
  position_control(8, L_NEUTRAL[0])
  position_control(9, L_NEUTRAL[1])
  position_control(10, L_NEUTRAL[2])
  time.sleep(DELAY)
  # spinWhileMoving()

  targets = np.concatenate((R_NEUTRAL, L_NEUTRAL), axis = None)
  position_control(2, R_NEUTRAL[0])
  position_control(3, R_NEUTRAL[1])
  position_control(4, R_NEUTRAL[2])
  time.sleep(DELAY)
  # spinWhileMoving()

def walkLogic(stepsToTake):
  stepsTaken = 0
  nextIsRight = True
  while stepsTaken < stepsToTake:
    if nextIsRight:
      rightStep()
      nextIsRight = not nextIsRight
    else:
      leftStep()
      nextIsRight = not nextIsRight
    stepsTaken += 1

if __name__ == "__main__":

  position_control(1, FIVETWELVE[0])
  position_control(2, R_NEUTRAL[0])
  position_control(3, R_NEUTRAL[1])
  position_control(4, R_NEUTRAL[2])
  position_control(5, FIVETWELVE[0])
  position_control(6, FIVETWELVE[0])
  position_control(7, FIVETWELVE[0])
  position_control(8, L_NEUTRAL[0])
  position_control(9, L_NEUTRAL[1])
  position_control(10, L_NEUTRAL[2])
  position_control(11, FIVETWELVE[0])
  position_control(12, FIVETWELVE[0])
  time.sleep(DELAY)
  walkLogic(3) # TODO make this user input
