#!/usr/bin/env python3

'''
Author: jia-yf19
Date: 2022-03-02 19:50:21
LastEditTime: 2022-03-19 20:21:36
Description: 遥控器
FilePath: /uav_nav/scripts/cmd_twist.py
'''

import rospy
import tf_conversions
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

g_init = False
g_started = False
g_position = [0,0,0]
g_eular = [0,0,0]

g_position_set = [0,0,0]
g_eular_set = [0,0,0]

def ChallengeStartedCallback(msg:Bool):
    global g_started
    if msg.data == True:
        g_started = True
        print("Challenge started!")

def PoseCallback(msg:PoseStamped):
    global g_init
    global g_started
    global g_eular
    global g_position

    if g_started:
        pose = msg.pose.position
        quat = msg.pose.orientation
        
        eular = tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        g_eular = [eular[0], eular[1], eular[2]]
        g_position = [pose.x, pose.y, pose.z]
        if not g_init:
            g_init = True
            for i in range(3):
                g_position_set[i] = g_position[i]
                g_eular_set[i] = g_eular[i]

def CmdCallback(msg:Twist):

    global g_started
    global g_eular
    global g_position

    if g_started:
        g_position_set[0] += msg.linear.x/2.0
        g_position_set[1] += msg.linear.y/2.0
        g_position_set[2] += msg.linear.z/2.0
        g_eular_set[0] += msg.angular.x/10.0
        g_eular_set[1] += msg.angular.y/10.0
        g_eular_set[2] += msg.angular.z/10.0

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    rospy.Subscriber("/red/challenge_started", Bool, ChallengeStartedCallback)
    rospy.Subscriber("/red/pose", PoseStamped, PoseCallback)
    rospy.Subscriber("/cmd_vel", Twist, CmdCallback)
    
    puber = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=10)
    msg = PoseStamped()

    print("Waiting...")
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if g_started and g_init:
            # msg.header.stamp = rospy.Time.now()
            # msg.header.frame_id = "keyboard_control"
            msg.pose.position.x = g_position_set[0]
            msg.pose.position.y = g_position_set[1]
            msg.pose.position.z = g_position_set[2]

            quat = tf_conversions.transformations.quaternion_from_euler(g_eular_set[0], g_eular_set[1], g_eular_set[2])
            msg.pose.orientation.x = quat[0]
            msg.pose.orientation.y = quat[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]

            puber.publish(msg)
        # print(">>>>>>>>>>>>>>>>>>>>")
        # print("Set pose:{:6.3f} {:6.3f} {:6.3f} orie:{:6.3f}".format(g_position_set[0], g_position_set[1], g_position_set[2], g_eular_set[2]))
        # print("Cur pose:{:6.3f} {:6.3f} {:6.3f} orie:{:6.3f}".format(g_position[0], g_position[1], g_position[2], g_eular[2]))
        rate.sleep()
