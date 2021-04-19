#!/usr/bin/env python3

import sys
import rospy
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

def test_control():
    speed_control(1, 128)
    speed_control(5, 128)
    speed_control(7, 128)
    speed_control(11, 128)
    position_control(1, 550)
    position_control(5, 550)
    position_control(7, 470)
    position_control(11, 470)

def speed_pos_control(ID, speed, position):
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    try:
        Command2 = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand())
        Command2 = ('', ID, 'Moving_Speed', speed)
        Command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand())
        Command('', ID, 'Goal_Position', position)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def speed_control(ID, speed):
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    try:
        Command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand())
        Command('', ID, 'Moving_Speed', speed)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def position_control(ID, position):
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    try:
        Command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand())
        Command('', ID, 'Goal_Position', position)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

#Don't think we will need this for now so ignore it.
def publisher():
    #rospy.init_node('sub', anonymous=True)
    message = msg.AX()
    pub = rospy.Publisher('left_ankle', msg.AX, queue_size=100)
    rospy.init_node('moverNode', anonymous=True)
    rate = rospy.Rate(10)
    message.Goal_Position = 400
    message.ID = 1
    pub.publish(message)
    rate.sleep()
    rospy.spin()

def motor_printer(motorSub):
    rate = rospy.Rate(10)
    rospy.loginfo("Motor 7 Position: %s" % (motorSub.dynamixel_state[0].present_position))

def motor_subscriber():
    rospy.init_node('motorSub', anonymous=True)
    posSub = rospy.Subscriber("dynamixel_workbench/dynamixel_state", msg.DynamixelStateList, motor_printer)
    rospy.spin()

def imu_printer(sub):
    rate = rospy.Rate(10)
    rospy.loginfo("Pitch: %s" % (sub.vector.x))
    rospy.loginfo("Roll: %s" % (sub.vector.y))
    rospy.loginfo("Yaw: %s" % (sub.vector.z))

def IMU_subscriber():
    rospy.init_node('imu_Node', anonymous=True)
    sub_x = rospy.Subscriber("navx_micro/euler", Vector3Stamped, imu_printer)
    rospy.spin()

if __name__ == "__main__":

    test_control()

    #to see examples of the functions running, just uncomment them

    #Gets data from IMU, you can use access it by using sub.vector.x etc.
    #IMU_subscriber()

    #Gets current position from motors, access with posSub.dynamixel_state[index].parameter, these are all listed in AX.msg
    #pos_subscriber()

    #Moves motor, arguments are motor ID and goal position (0, 1023)
    #position_control(1, 512)

    #Sets motor speed, arguments are motor ID and goal speed (0, 1023)
    #speed_control(1, 512)


    #Me attempting to combine the above functions, it appears to work.
    #(ID, Speed, Position)
    #speed_pos_control(1, 512, 512)
