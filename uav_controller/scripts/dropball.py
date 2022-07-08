#!/usr/bin/env python3

from this import d
import rospy
from std_msgs.msg import String, Bool, Float32
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Transform, Twist
import numpy as np
import tf_conversions as tfc
from copy import deepcopy
from TagPos import TagPos
import time


class DBPS:
    def __init__(self):
        # record uav pose info
        self.RedPose = PoseStamped()
        # record ball pose info
        self.lastballPose = PoseStamped()
        self.ballPose = PoseStamped()
        # record isStart?
        self.IsStarted = False
        # ball velocite
        self.BallV = {"x":0.0, "y":0.0, "z":0.0}
        # target point
        self.target = [0, 0, 2]
        # self.target = [12.5, -3.0, 2.0]
        # self.target = [7, -7.5, 2.0]
        # self.target = [6, 7.5, 2.0]
        # can attack point
        self.attack = [0.0, 0.0, 0.0]

        # pubish uav tracker
        self.Puber_tracker = rospy.Publisher("/red/position_hold/trajectory",
                                            MultiDOFJointTrajectoryPoint,
                                            queue_size=1)
        # throw ball
        self.Puber_ThrowBall = rospy.Publisher("/red/uav_magnet/gain",
                                               Float32,
                                               queue_size=1)

        # Subscribe ball status
        self.Suber_BStatus = rospy.Subscriber("/red/ball/pose", PoseStamped,
                                             self.BallPoseCallback)                                                      
        # Subscribe challenge_started
        self.Suber_Started = rospy.Subscriber("/red/challenge_started", Bool,
                                              self.StartedCallback)
        # Subscribe pose
        self.Suber_Pose = rospy.Subscriber("/red/pose", PoseStamped,
                                           self.PoseCallback)

    def PubPose(self, pose_list):
        msg = MultiDOFJointTrajectoryPoint()
        msg1 = Transform()
        msg1.translation.x = pose_list[0]
        msg1.translation.y = pose_list[1]
        msg1.translation.z = pose_list[2]
        msg1.rotation.x = pose_list[3]
        msg1.rotation.y = pose_list[4]
        msg1.rotation.z = pose_list[5]
        msg1.rotation.w = pose_list[6]

        msg.transforms.append(msg1)
        msg2 = Twist()
        msg.velocities.append(msg2)
        msg3 = Twist()
        msg.accelerations.append(msg3)

        self.Puber_tracker.publish(msg)

    def ThrowBall(self):
        msg = Float32()
        msg.data = 0.0
        self.Puber_ThrowBall.publish(msg)

    def BallPoseCallback(self, ball_pose):
        # update ball pose and velocity
        self.lastballPose = deepcopy(self.ballPose)
        self.ballPose = ball_pose
        delta_time = self.ballPose.header.stamp.secs - self.lastballPose.header.stamp.secs + (self.ballPose.header.stamp.nsecs - self.lastballPose.header.stamp.nsecs) * 1e-9
        self.BallV['x'] = (self.ballPose.pose.position.x - self.lastballPose.pose.position.x) / delta_time
        self.BallV['y'] = (self.ballPose.pose.position.y - self.lastballPose.pose.position.y) / delta_time
        self.BallV['z'] = (self.ballPose.pose.position.z - self.lastballPose.pose.position.z) / delta_time
        # calculate landing point according to the projectile equation
        if self.target[0] == 12.5:
            self.attack[0] = self.target[0] - self.ballPose.pose.position.x
            self.attack[1] = self.target[1]
            self.attack[2] = self.ballPose.pose.position.z - 4.9 * (self.attack[0] / self.BallV['x'])**2 + self.attack[0] * (self.BallV['z'] / self.BallV['x'])
        else:
            self.attack[0] = self.target[0]
            self.attack[1] = abs(self.target[1] - self.ballPose.pose.position.y)
            self.attack[2] = self.ballPose.pose.position.z - 4.9 * (self.attack[1] / abs(self.BallV['y']))**2 + self.attack[1] * (self.BallV['z'] / abs(self.BallV['y']))

    def StartedCallback(self, is_started):
        # get the challenge_started
        self.IsStarted = is_started

    def PoseCallback(self, red_pose):
        self.RedPose = red_pose
        # rospy.loginfo(self.RedPose)

    def IsArrived(self, pose, threshold=0.5):
        delta = 0
        pos_c = self.RedPose.pose.position
        quat_c = self.RedPose.pose.orientation

        delta += abs(pos_c.x - pose[0])
        delta += abs(pos_c.y - pose[1])
        delta += abs(pos_c.z - pose[2])

        euler_c = tfc.transformations.euler_from_quaternion(
            [quat_c.x, quat_c.y, quat_c.z, quat_c.w])
        euler_t = tfc.transformations.euler_from_quaternion(pose[3:])

        for i in range(3):
            delta += abs(euler_t[i] - euler_c[i])

        if delta < threshold:
            return True
        else:
            return False

    def IsArrived_DropBall_Position(self, threshold=0.1):
        delta = abs(self.target[2] - self.attack[2])
        if delta < threshold:
            return True
        else:
            return False

    # read the ARcode position and init some key point
    def ARcode_initPoint(self, tp_pose):
        # read the ARcode position to modify the self.target
        self.target[0] = tp_pose.pose.position.x
        self.target[1] = tp_pose.pose.position.y
        self.target[2] = tp_pose.pose.position.z

        # rospy.loginfo(tp_pose)

        # when the ARcode is on the forward wall
        if 12 < self.target[0] < 13:
            self.target[0] = 12.5
            Pose1 = np.array([self.target[0] - 2.5, self.target[1], 4, 0, 0, 0, 0])
            Pose2 = np.array([self.target[0] - 2.5, self.target[1], 1, 0, 0, 0, 0])
            Pose3 = np.array([self.target[0] - 0.5, self.target[1], 4.5, 0, 0, 0, 0])
            return Pose1, Pose2, Pose3
        # when the ARcode is on the left wall
        elif 7 < self.target[1] < 8:
            self.target[1] = 7.5
            Pose1 = np.array([self.target[0], self.target[1] - 2.5, 4, 0, 0, 0, 0])
            Pose2 = np.array([self.target[0], self.target[1] - 2.5, 1, 0, 0, 0, 0])
            Pose3 = np.array([self.target[0], self.target[1] - 0.5, 4.5, 0, 0, 0, 0])
            return Pose1, Pose2, Pose3
        # when the ARcode is on the left wall
        elif -8 < self.target[1] < -7:
            self.target[1] = -7.5
            Pose1 = np.array([self.target[0], self.target[1] + 2.5, 4, 0, 0, 0, 0])
            Pose2 = np.array([self.target[0], self.target[1] + 2.5, 1, 0, 0, 0, 0])
            Pose3 = np.array([self.target[0], self.target[1] + 0.5, 4.5, 0, 0, 0, 0])
            return Pose1, Pose2, Pose3
        else:
            rospy.loginfo("read the ARcode ERROR!")
            # rospy.loginfo(self.target)
            return None, None, None
        

def Main_Dropball():
    Kp_z = 0.0337
    Kp_y = 0.0337
    rospy.init_node("uav_dropball", anonymous=True)
    rate = rospy.Rate(24)
    dbps = DBPS()
    tp = TagPos()
    # to judge whether ball was been dropped
    is_Drop_ball = False
    is_Ready_to_dropball = False
    safePoint = np.array([2,0,4,0,0,0,0])
    # the competition is start or not (delete later)
    while True:
        if dbps.IsStarted:
            break
    time.sleep(1)
    dz = tp.Bias_z
    dy = tp.Bias_y
    print("z,y axis delta:", dz, dy)
    pose = [dbps.RedPose.pose.position.x, dbps.RedPose.pose.position.y - Kp_y * dy, dbps.RedPose.pose.position.z - Kp_z * dz, 0, 0, 0, 0]
    print("pose", pose)
    while not dbps.IsArrived(pose):
        dbps.PubPose(pose)
        rate.sleep()
    # to read the ARcode position
    # Pose1 = None
    # rospy.loginfo(Pose1)
    while not tp.IsOK:
        # TODO
        pose = [dbps.RedPose.pose.position.x + 1, dbps.RedPose.pose.position.y, dbps.RedPose.pose.position.z, 0, 0, 0, 0]
        while not dbps.IsArrived(pose):
            dbps.PubPose(pose)
            rate.sleep()

        Pose1,Pose2,Pose3 = dbps.ARcode_initPoint(tp.TagPose)
        rate.sleep()

    # main loop
    while(is_Drop_ball is False):
        # Pose1 is the init point
        while not dbps.IsArrived(Pose1):
            dbps.PubPose(Pose1)
        # to fly lower
        while not dbps.IsArrived(Pose2):
            dbps.PubPose(Pose2)
        # the final curve to throw the ball
        dbps.PubPose(Pose3)
        while not dbps.IsArrived_DropBall_Position() and not dbps.IsArrived(Pose3):
            dbps.PubPose(Pose3)
            if dbps.IsArrived_DropBall_Position() is True:
                is_Ready_to_dropball = True
                break

        if is_Ready_to_dropball is True:
            dbps.ThrowBall()
            rospy.loginfo("throw the ball")
            is_Drop_ball = True
            dbps.PubPose(safePoint)
        else: continue

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    # rospy.init_node("uav_dropball", anonymous=True)
    # rate = rospy.Rate(24)
    # dbps = DBPS()
    # # to judge whether ball was been dropped
    # is_Drop_ball = False
    # # the competition is start or not (delete later)
    # while True:
    #     if dbps.IsStarted:
    #         break
    # start_pose = np.array([0.0, 0.0, 4.5, 0, 0, 0, 0])
    # to_pose = np.array([9.0, -3.0, 4.5, 0, 0, 0, 0])
    # middle_pose = np.array([10.0, -3.0, 1, 0, 0, 0, 0])
    # end_pose = np.array([12, -3, 4.5, 0, 0, 0, 0])

    
    # while not dbps.IsArrived(start_pose):
    #     dbps.PubPose(start_pose)

    # while not dbps.IsArrived(to_pose):
    #     dbps.PubPose(to_pose)

    # while not dbps.IsArrived(middle_pose):
    #     dbps.PubPose(middle_pose)

    # dbps.PubPose(end_pose)
    # while not dbps.IsArrived_DropBall_Position():
    #     dbps.PubPose(end_pose)

    # dbps.ThrowBall()
    # rospy.loginfo("throw the ball")

    # while not dbps.IsArrived(start_pose):
    #     dbps.PubPose(start_pose)

    # while not rospy.is_shutdown():
    #     rate.sleep()

    Main_Dropball()