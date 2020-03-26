#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import rospy
from std_msgs.msg import Bool
import numpy as np
from darknet import detect_np,load_net,load_meta,check_detected_result_image

###########################
# Webcam_Object_detection #
###########################
class Webcam_Object_detection():

    ##################
    # Initialization #
    ##################
    def __init__(self):

        rospy.init_node('Webcam_Object_detection', anonymous=True)
        r = rospy.Rate(50)  # 50hz

        #initialize webcam
        self.cap = cv2.VideoCapture(0)

        #load yolo_v3 parameters
        self.net = load_net("/home/xxxx/darknet/cfg/yolov3-tiny.cfg", "/home/xxxx/darknet/cfg/yolov3-tiny.weights", 0)
        self.meta = load_meta("/home/xxxx/darknet/cfg/coco.data")

        self.obstacle_label = ["person"]
        self.obstacle_ratio = 0.1
        self.obstacle_frame_th = 3
        self.obstacle_cancel_frame_th = 5
        ret, frame = self.cap.read()
        self.imsize = frame.shape[0]*frame.shape[1]
        self.obstacle_cnt = 0
        self.obstacle_flg = False

        #initialize publisher
        self.object_detect_publisher = rospy.Publisher("/object_detection", Bool, queue_size=50)

    def detect_object(self,event):
        ret, frame = self.cap.read()

        k = cv2.waitKey(1)
        r = detect_np(self.net, self.meta, frame)
        result_im = check_detected_result_image(frame, r)
        cv2.imshow('Image', result_im )
        cv2.waitKey(1)

        #check obstacle
        flg = False
        for indx in range(len(r)):  #Check detected objects
            for indx2 in range(len(self.obstacle_label)):   #check label name
                #Check Object Label
                if r[indx][0] == self.obstacle_label[indx2]:
                    ratio = r[indx][2][2]*r[indx][2][3]/self.imsize
                    if ratio > self.obstacle_ratio: #check object size (ratio per frame size)
                        flg = True
                        break
            if flg == True:
                break
        if flg == True:
            self.obstacle_cnt = min(self.obstacle_cnt + 1,self.obstacle_cancel_frame_th)
        else:
            self.obstacle_cnt = max(self.obstacle_cnt - 1, 0)

        if self.obstacle_cnt > self.obstacle_frame_th:
            self.obstacle_flg = True
        elif self.obstacle_cnt <= 0:
            self.obstacle_flg = False

        self.object_detect_publisher.publish(self.obstacle_flg)
        print(self.obstacle_cnt)


if __name__ == "__main__":

    test = Webcam_Object_detection()
    rospy.Timer(rospy.Duration(0.1), test.detect_object)
    rospy.spin()

    

