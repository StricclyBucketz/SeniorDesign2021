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

DELAY = 0.020
TOLERANCE = 10 # %tolerance = TOLERANCE / 1023 (In this case ~0.3%)
BASE_MOTOR_SPEED = 128 # Sets the speed for the servo moving the farthest in a state change.
STATIC = np.array([512])    # Default for not {2,3,4,8,9,10}
# TWO_POS = np.array([622,572])       # Default for servo 2, its extended state
TWO_POS = np.array([647,577])       # Default for servo 2, its extended state
#TWO_POS = np.array([667,597])
# THREE_POS = np.array([262,237])
THREE_POS = np.array([237,287])
# FOUR_POS = np.array([412,362])      # Default for servo 4, its extended state
FOUR_POS = np.array([367,277])      # Default for servo 4, its extended state
#FOUR_POS = np.array([345,255])
# EIGHT_POS = np.array([402,452])     # Default for servo 8, its extended state
EIGHT_POS = np.array([377,447])     # Default for servo 8, its extended state
#EIGHT_POS = np.array([327,427])
# NINE_POS = np.array([762,787])
NINE_POS = np.array([770,820])
# TEN_POS = np.array([612,662])       # Default for servo 10, its extended state
TEN_POS = np.array([647,737,670])       # Default for servo 10, its extended state
#TEN_POS = np.array([679,769])
# FEETHIPS = np.array([487,537])      # When not 512, both hips are one, both feet are other
FEETHIPS = np.array([492,532])      # When not 512, both hips are one, both feet are other
TWIST = np.array([462,562])
# TWIST = np.array([487, 537])
# SPEED = np.array([BASE_MOTOR_SPEED, math.ceil(BASE_MOTOR_SPEED / 2), math.ceil(BASE_MOTOR_SPEED / 4)]) # IF first/last step: {2,4,8,10},{1,5,7,11} ELSE: {all}, {}
SPEED = np.array([BASE_MOTOR_SPEED, math.ceil(0.9*BASE_MOTOR_SPEED), math.ceil(0.7*BASE_MOTOR_SPEED), math.ceil(0.5*BASE_MOTOR_SPEED), math.ceil(0.4*BASE_MOTOR_SPEED), math.ceil(0.2*BASE_MOTOR_SPEED)]) # use PosSpeed

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




def set_speeds():
  speed_control(1, BASE_MOTOR_SPEED*2)
  speed_control(2, BASE_MOTOR_SPEED)
  speed_control(3, BASE_MOTOR_SPEED)
  speed_control(4, BASE_MOTOR_SPEED)
  speed_control(5, BASE_MOTOR_SPEED)
  speed_control(6, BASE_MOTOR_SPEED)
  speed_control(7, BASE_MOTOR_SPEED)
  speed_control(8, BASE_MOTOR_SPEED)
  speed_control(9, BASE_MOTOR_SPEED)
  speed_control(10, BASE_MOTOR_SPEED)
  speed_control(11, BASE_MOTOR_SPEED)
  speed_control(12, BASE_MOTOR_SPEED)
  speed_control(13, BASE_MOTOR_SPEED)
  speed_control(14, BASE_MOTOR_SPEED)
  speed_control(15, BASE_MOTOR_SPEED)
  speed_control(16, BASE_MOTOR_SPEED)
  speed_control(17, BASE_MOTOR_SPEED)
  speed_control(18, BASE_MOTOR_SPEED)
  time.sleep(3)

#State 31
def walk_ready():
  position_control(1, 508)
  position_control(2, 681)
  position_control(3, 143)
  position_control(4, 284)
  position_control(5, 508)
  position_control(6, 512)
  #time.sleep(1)
  position_control(7, 515)
  position_control(8, 342)
  position_control(9, 880)
  position_control(10, 739)
  position_control(11, 515)
  position_control(12, 512)
  position_control(13, 512)
  position_control(14, 574)
  position_control(15, 470)
  position_control(16, 512)
  position_control(17, 449)
  position_control(18, 553)
  time.sleep(1)

#State 32
def F_S_L():

  # position_control(5, 500)
  # time.sleep(0.055)
  # position_control(1, 365)
  # position_control(5, 578)
  # position_control(7, 545)
  # position_control(11, 575)
  # time.sleep(0.055)
  # #Substate 32.1
  speed_control(1, BASE_MOTOR_SPEED)
  speed_control(3, math.ceil(0.2*BASE_MOTOR_SPEED))
  speed_control(4, math.ceil(0.2*BASE_MOTOR_SPEED))
  speed_control(5, BASE_MOTOR_SPEED)
  speed_control(7, BASE_MOTOR_SPEED)
  speed_control(8, math.ceil(0.2*BASE_MOTOR_SPEED))
  speed_control(9, math.ceil(0.4*BASE_MOTOR_SPEED))
  speed_control(10, math.ceil(0.895*BASE_MOTOR_SPEED))
  speed_control(11, math.ceil(0.649*BASE_MOTOR_SPEED))
  speed_control(13, math.ceil(0.632*BASE_MOTOR_SPEED))
  speed_control(16, math.ceil(0.632*BASE_MOTOR_SPEED))

  position_control(1, 513)
  position_control(2, 681)
  position_control(3, 158)
  position_control(4, 299)
  position_control(5, 513)
  
  position_control(7, 520)
  position_control(8, 343)
  position_control(9, 864)
  position_control(10, 724)
  position_control(11, 520)
  
  position_control(13, 537)
  
  position_control(16, 486)
  time.sleep(DELAY)

  # #Substate 32.2
  position_control(1, 518)
  position_control(2, 679)
  position_control(3, 162)
  position_control(4, 301)
  position_control(5, 518)
  
  position_control(7, 525)
  position_control(8, 345)
  position_control(9, 860)
  position_control(10, 721)
  position_control(11, 525)
  # time.sleep(DELAY)

  # #Substate 32.3
  position_control(1, 522)
  position_control(2, 677)
  position_control(3, 166)
  position_control(4, 303)
  position_control(5, 522)
  
  position_control(7, 529)
  position_control(8, 348)
  position_control(9, 855)
  position_control(10, 719)
  position_control(11, 529)
  # # time.sleep(DELAY)

  # #Substate 32.4
  position_control(1, 525)
  position_control(2, 675)
  position_control(3, 171)
  position_control(4, 305)
  position_control(5, 525)
  
  position_control(7, 533)
  position_control(8, 350)
  position_control(9, 849)
  position_control(10, 716)
  position_control(11, 533)
  time.sleep(DELAY)

  # #Substate 32.5
  position_control(1, 528)
  position_control(2, 672)
  position_control(3, 176)
  position_control(4, 308)
  position_control(5, 520)
  
  position_control(7, 536)
  position_control(8, 344)
  position_control(9, 862)
  position_control(10, 722)
  position_control(11, 539)
  time.sleep(DELAY)

  # #Substate 32.6
  position_control(1, 529)
  position_control(2, 671)
  position_control(3, 179)
  position_control(4, 307)
  position_control(5, 504)
  
  position_control(7, 539)
  position_control(8, 330)
  position_control(9, 889)
  position_control(10, 736)
  position_control(11, 548)
  time.sleep(DELAY)

  #Substate 32.7
  position_control(1, 508)
  position_control(2, 670)
  position_control(3, 166)
  position_control(4, 264)
  position_control(5, 497)

  position_control(7, 500)
  position_control(8, 324)
  position_control(9, 930)
  position_control(10, 771)
  position_control(11, 555)
  # time.sleep(DELAY)

#State 33
def F_S_L_MID():

  #Substate 33.1
  position_control(1, 529)
  position_control(2, 676)
  position_control(3, 180)
  position_control(4, 308)
  position_control(5, 504)

  #position_control(7, 539)
  position_control(8, 350)
  position_control(9, 886)
  position_control(10, 761)
  position_control(11, 555)

  position_control(13, 535)

  position_control(16, 484)
  # time.sleep(DELAY)

  # #Substate 33.2
  # position_control(1, 528)
  # position_control(2, 682)
  # position_control(3, 177)
  # position_control(4, 312)
  # position_control(5, 520)
  #
  # position_control(7, 536)
  # position_control(8, 379)
  # position_control(9, 851)
  # position_control(10, 755)
  # position_control(11, 538)
  #
  # position_control(13, 548)
  #
  # position_control(16, 497)
  # # time.sleep(DELAY)

  # #Substate 33.3
  # position_control(1, 525)
  # position_control(2, 686)
  # position_control(3, 173)
  # position_control(4, 310)
  # position_control(5, 525)
  #
  # position_control(7, 533)
  # position_control(8, 393)
  # position_control(9, 833)
  # position_control(10, 751)
  # position_control(11, 533)
  #
  # position_control(13, 560)
  #
  # position_control(16, 509)
  # # time.sleep(DELAY)
  # #
  # #Substate 33.4
  # position_control(1, 522)
  # position_control(2, 688)
  # position_control(3, 167)
  # position_control(4, 308)
  # position_control(5, 522)
  #
  # position_control(7, 529)
  # position_control(8, 390)
  # position_control(9, 839)
  # position_control(10, 755)
  # position_control(11, 529)
  #
  # position_control(13, 571)
  #
  # position_control(16, 520)
  # # time.sleep(DELAY)

  # #Substate 33.5
  # position_control(1, 518)
  # position_control(2, 692)
  # position_control(3, 164)
  # position_control(4, 308)
  # position_control(5, 518)
  #
  # position_control(7, 525)
  # position_control(8, 384)
  # position_control(9, 847)
  # position_control(10, 756)
  # position_control(11, 525)
  #
  # position_control(13, 580)
  #
  # position_control(16, 529)
  # # time.sleep(DELAY)

  # #Substate 33.6
  # position_control(1, 513)
  # position_control(2, 697)
  # position_control(3, 162)
  # position_control(4, 312)
  # position_control(5, 513)
  #
  # position_control(7, 520)
  # position_control(8, 377)
  # position_control(9, 854)
  # position_control(10, 756)
  # position_control(11, 520)
  #
  # position_control(13, 587)
  #
  # position_control(16, 536)
  # # time.sleep(DELAY)


  #Substate 33.7
  position_control(1, 508)
  position_control(2, 701)
  position_control(3, 150)
  position_control(4, 303)
  position_control(5, 508)

  #position_control(7, 515)
  position_control(8, 369)
  position_control(9, 873)
  position_control(10, 767)
  position_control(11, 555)

  position_control(13, 592)

  position_control(16, 541)
  # time.sleep(DELAY)

#State 34
def F_S_R():
  position_control(1, 483)
  position_control(2, 642)
  position_control(3, 249)
  position_control(4, 352)
  position_control(5, 471)

  position_control(7, 493)
  position_control(8, 410)
  position_control(9, 714)
  position_control(10, 642)
  position_control(11, 526)

  position_control(13, 537)

  position_control(16, 486)
  time.sleep(DELAY)

#State 35
def F_S_R_MID():
  position_control(1, 508)
  position_control(2, 597)
  position_control(3, 292)
  position_control(4, 341)
  position_control(5, 508)

  position_control(7, 515)
  position_control(8, 379)
  position_control(9, 731)
  position_control(10, 635)
  position_control(11, 515)

  position_control(13, 481)

  position_control(16, 430)
  time.sleep(DELAY)

#State 38
def F_M_R():
  position_control(1, 484)
  position_control(2, 700)
  # position_control(3, 249)
  # position_control(4, 335)
  position_control(3, 106)
  position_control(4, 249)
  position_control(5, 473)

  position_control(7, 494)
  position_control(8, 353)
  position_control(9, 843)
  position_control(10, 730)
  position_control(11, 527)

  position_control(13, 565)

  position_control(16, 514)
  time.sleep(DELAY)

#State 39
def F_M_R_MID():
  position_control(1, 508)
  position_control(2, 654)
  position_control(3, 164)
  position_control(4, 262)
  position_control(5, 508)

  position_control(7, 515)
  position_control(8, 322)
  position_control(9, 859)
  position_control(10, 714)
  position_control(11, 515)

  position_control(13, 487)

  position_control(16, 436)
  time.sleep(DELAY)

#State 40
def F_E_L():
  position_control(1, 530)
  position_control(2, 670)
  position_control(3, 181)
  position_control(4, 302)
  position_control(5, 497)

  position_control(7, 540)
  position_control(8, 324)
  # position_control(9, 774)
  # position_control(10, 680)
  position_control(9, 916)
  position_control(10, 765)
  position_control(11, 552)

  position_control(13, 523)

  position_control(16, 472)
  time.sleep(DELAY)

#State 41
def F_E_L_MID():
  position_control(1, 508)
  position_control(2, 681)
  position_control(3, 157)
  position_control(4, 298)
  position_control(5, 508)

  position_control(7, 515)
  position_control(8, 342)
  position_control(9, 866)
  position_control(10, 725)
  position_control(11, 515)

  position_control(13, 537)

  position_control(16, 486)
  time.sleep(DELAY)



if __name__ == "__main__":

  set_speeds()
  walk_ready()
  F_S_L()
  F_S_L_MID()
  # F_M_R()
  # F_M_R_MID()
  # F_E_L()
  # F_E_L_MID()
  #walk_ready()
