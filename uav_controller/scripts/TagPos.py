#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import tf
import tf2_ros
# from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Bool, Float32
from cv_bridge import CvBridge
import numpy as np


# class PoseNumpy:

#     def __init__(self, pos=None, ori=None):
#         if pos is None:
#             self.Position = np.array([0, 0, 0])
#         else:
#             self.Position = pos

#         if ori is None:
#             self.Orientation = np.array([1, 0, 0, 0])
#         else:
#             self.Orientation = ori

#     def FromRos(self, msg: Pose):
#         self.Position[0] = msg.position.x
#         self.Position[1] = msg.position.y
#         self.Position[2] = msg.position.z

#         self.Orientation[0] = msg.orientation.x
#         self.Orientation[1] = msg.orientation.y
#         self.Orientation[2] = msg.orientation.z
#         self.Orientation[3] = msg.orientation.w


class TagPos:
    ratio_x = 0.252
    ratio_y = 0.252
    ratio_z = 0.252

    def __init__(self):
        self.IsOK = False
        self.k = 1
        self.x = 0
        self.y = 0
        self.Bias_z = 0.0
        self.Bias_y = 0.0
        self.ColorImg = np.zeros(1)  # 彩色图像
        self.MarkerInfo = Marker()  # Marker信息
        self.TagPose = PoseStamped()  # tag的pose信息，绝对坐标
        self.RedPose = PoseStamped()  # red的pose信息，绝对坐标
        self.TFListener = tf.TransformListener()  # tf监听器
        # publish pose
        self.Puber_Pose = rospy.Publisher("/red/tracker/input_pose",
                                          PoseStamped,
                                          queue_size=1)
        # 获取彩色图像
        self.Suber_Color = rospy.Subscriber("/red/camera/color/image_raw",
                                            Image, self.ColorCallback)
        # 获取Marker信息
        self.Suber_Marker = rospy.Subscriber("/visualization_marker", Marker,
                                             self.MarkerCallback)
        # 获取当前坐标
        self.Suber_Pose = rospy.Subscriber("/red/pose", PoseStamped,
                                           self.PoseCallback)
    
    def Mean(self, new_pose):
        pass
        self.TagPose.pose.position.x = (1 - 1 / self.k) * self.TagPose.pose.position.x + 1 / self.k * new_pose.pose.position.x
        self.TagPose.pose.position.y = (1 - 1 / self.k) * self.TagPose.pose.position.y + 1 / self.k * new_pose.pose.position.y
        self.TagPose.pose.position.z = (1 - 1 / self.k) * self.TagPose.pose.position.z + 1 / self.k * new_pose.pose.position.z
        # self.TagPose.pose.orientation.x = (1 - 1 / self.k) * self.TagPose.pose.orientation.x + 1 / self.k * new_pose.pose.orientation.x
        # self.TagPose.pose.orientation.y = (1 - 1 / self.k) * self.TagPose.pose.orientation.y + 1 / self.k * new_pose.pose.orientation.y
        # self.TagPose.pose.orientation.z = (1 - 1 / self.k) * self.TagPose.pose.orientation.z + 1 / self.k * new_pose.pose.orientation.z
        # self.TagPose.pose.orientation.w = (1 - 1 / self.k) * self.TagPose.pose.orientation.w + 1 / self.k * new_pose.pose.orientation.w
        self.TagPose.pose.orientation.x = new_pose.pose.orientation.x
        self.TagPose.pose.orientation.y = new_pose.pose.orientation.y
        self.TagPose.pose.orientation.z = new_pose.pose.orientation.z
        self.TagPose.pose.orientation.w = new_pose.pose.orientation.w

    def GetTagPose(self):
        """
        get the tf between camera and red
        """
        marker_id = self.MarkerInfo.header.frame_id
        
        # print(marker_id)
        # if self.TFListener.frameExists(
        #         "/world") and self.TFListener.frameExists("/red/camera"):
        try:
            t = self.TFListener.getLatestCommonTime("/world", "/red/camera")
            p = PoseStamped()
            p.header.frame_id = marker_id
            p.pose = self.MarkerInfo.pose
            p.pose.position.x /= self.ratio_x
            p.pose.position.y /= self.ratio_y
            p.pose.position.z /= self.ratio_z
            p_in_world = self.TFListener.transformPose("/world", p)
            self.Mean(p_in_world)
            # print("Position of marker in world:\n==========================\n", self.TagPose)
            self.IsOK = True
            pass
        except Exception as e:
            self.IsOK = False
            print(e)
        
        self.k += 1
        if self.k > 10000:
            self.k = 10000


    def ColorCallback(self, color):
        """
        get the color image
        """
        try:
            np_img = CvBridge().imgmsg_to_cv2(color)
            self.ColorImg = np_img
            if not self.IsOK:
                # print(self.ColorImg.shape)
                idx = np.where(self.ColorImg[:, :, 0] + self.ColorImg[:, :, 1] + self.ColorImg[:, :, 2] < 20)
                # print(idx)
                # the pos in img

                if len(idx[0]) != 0:
                    self.x = np.mean(idx[0])
                    self.y = np.mean(idx[1])

                self.BiasMiddle()
                # print(self.Bias_z, self.Bias_y)
                # print(self.x, self.y, idx[0].shape[0])
        except Exception as e:
            print(e)
    
    def BiasMiddle(self):
        """
        check if the tag is in the middle of the image
        """
        THR = 10
        dx = self.x - self.ColorImg.shape[0] / 2
        dy = self.y - self.ColorImg.shape[1] / 2
        if abs(dx) < THR:
            self.Bias_z = 0
        else:
            self.Bias_z = dx
        if abs(dy) < THR:
            self.Bias_y = 0
        else:
            self.Bias_y = dy

    def MarkerCallback(self, marker: Marker):
        self.MarkerInfo = marker
        self.GetTagPose()
        # print(self.MarkerInfo.header.frame_id)

    def PoseCallback(self, red_pose):
        self.RedPose = red_pose


if __name__ == '__main__':
    rospy.init_node("uav_controller", anonymous=True)
    tp = TagPos()
    # msg = Marker()
    # msg.pose.position
    # msg.pose.orientation
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # tp.GetTagPose()
        # rospy.loginfo("x: %f" % to_pose[0])
        # psuber.PubPose(to_pose)
        rate.sleep()
