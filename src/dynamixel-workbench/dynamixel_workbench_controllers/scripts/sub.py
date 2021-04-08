#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs import msg
from dynamixel_workbench_msgs import srv
import time

from geometry_msgs.msg import Quaternion, Vector3, Vector3Stamped
from sensor_msgs.msg import Imu


   
def servicer():
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    try:
        Command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand())
        Command('', 7, 'Goal_Position', 580)
        print(Command.comm_result)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    
   
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
    
def pos_printer(sub):
    rate = rospy.Rate(10)  
    rospy.loginfo("Motor 7 Position: %s" % (posSub.dynamixel_state[0].present_position))
    rospy.spin()
    rate.sleep()

        
def pos_subscriber():
    #message = msg.AX()
    rospy.init_node('posSub', anonymous=True)
    posSub = rospy.Subscriber("dynamixel_workbench/dynamixel_state", msg.DynamixelStateList, pos_printer)
    rospy.spin()

def imu_printer(sub):
    rate = rospy.Rate(10)  
    rospy.loginfo("%s" % (sub.vector.x))

def IMU_subscriber():
    #message = msg.AX()
    rospy.init_node('imuNode', anonymous=True)
    sub = rospy.Subscriber("navx_micro/euler", euler_msg, imu_printer)
    rospy.spin()

if __name__ == "__main__":
    pos_subscriber()
