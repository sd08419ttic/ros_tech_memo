#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import signal
import pandas as pd

# import for ros function
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header
from Tkinter import *
from nav_msgs.msg import Odometry

#############################
# Simple Odometry simulator #
#############################
class Simple_odom_simulator():

    ##################
    # Initialization #
    ##################
    def __init__(self):

        rospy.init_node('Simple_Odometry_Simlator', anonymous=True)
        r = rospy.Rate(50)  # 50hz

        #Save path_plan flag
        self.save_path_as_csv = True

        #Cmd_vel receive flag (auto control)
        self.subscribe_cmd_vel = True

        #Initialize odometry header
        self.odom_header = Header()
        self.odom_header.seq = 0
        self.odom_header.stamp = rospy.Time.now()
        self.odom_header.frame_id = "map"

        # Initialize pose info
        self.sim_pose = Pose()
        self.sim_pose.position.x = 0.0
        self.sim_pose.position.y = 0.0
        self.sim_pose.position.z = 0.0
        self.sim_pose.orientation.x = 0.0
        self.sim_pose.orientation.y = 0.0
        self.sim_pose.orientation.z = 0.0

        # initialize twist info
        self.sim_twist = Twist()

        # Initialize odometry info
        self.sim_odom = Odometry()
        self.sim_odom.header = self.odom_header
        self.sim_odom.child_frame_id = "base_link"
        self.sim_odom.pose.pose = self.sim_pose
        self.sim_odom.twist.twist = self.sim_twist

        #configuration for tkinter
        self.root = Tk()
        self.root.option_add('*font', ('FixedSys', 14))
        self.buffer = StringVar()
        self.buffer.set('')
        Label(self.root, text='Simple Odometry Simulator').pack()
        self.a = Label(self.root, textvariable=self.buffer)
        self.a.pack()
        self.a.bind('<Any-KeyPress>', self.cb_keyevent)

        #initialize publisher
        self.emu_odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)

        #initialize TF
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.map_broadcaster = tf.TransformBroadcaster()

        #set callback for ctrl and c
        signal.signal(signal.SIGINT, self.ctr_c_interruption)

        if self.save_path_as_csv == True:
            self.path_dict = {}

        if self.subscribe_cmd_vel == True:
            self.cmdvel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cb_get_cmdvel_subscriber)
            self.cmdvel_linear_x = 0.0
            self.cmdvel_linear_y = 0.0
            self.cmdvel_angular_z = 0.0

    ###########################
    # Bind Key board callback #
    ###########################
    def cb_keyevent(self,event):
        key = event.keysym

        #Update User request Speed from Key board
        if key=="Up":
            print("Up")
            self.sim_twist.linear.x = self.sim_twist.linear.x + 0.1 #m/sec
            pass
        elif key=="Down":
            print("Down")
            self.sim_twist.linear.x = self.sim_twist.linear.x - 0.1 #m/sec
            pass
        elif key == "Left":
            self.sim_twist.angular.z = self.sim_twist.angular.z + 0.1 #rad/sec
            print("Left")
            pass
        elif key == "Right":
            self.sim_twist.angular.z = self.sim_twist.angular.z - 0.1  #rad/sec
            print("Right")
            pass
        elif key == "Escape":
            self.sim_twist.linear.x = 0.0  # m/sec
            self.sim_twist.angular.z = 0.0 #rad/sec
            print("Escape")

        #Min-Max GUARD
        if self.sim_twist.linear.x < -5.0:
            self.sim_twist.linear.x = -5.0
        elif self.sim_twist.linear.x > 5.0:
            self.sim_twist.linear.x = 5.0

        if self.sim_twist.angular.z < -3.14:
            self.sim_twist.angular.z = -3.14
        elif self.sim_twist.angular.z > 3.14:
            self.sim_twist.angular.z = 3.14


    #############################################
    # Update odometry form User request cmd_vel #
    #############################################
    def update_odom(self):
        #Update Vehicle Pose

        sampletime = 0.1    #calculate by 100msec
        e = tf.transformations.euler_from_quaternion((self.sim_pose.orientation.x, self.sim_pose.orientation.y, self.sim_pose.orientation.z, self.sim_pose.orientation.w))
        yaw_euler = e[2]


        #update pose from user request
        if self.subscribe_cmd_vel == False:
            self.sim_pose.position.x = self.sim_pose.position.x + self.sim_twist.linear.x*sampletime*math.cos(yaw_euler)
            self.sim_pose.position.y = self.sim_pose.position.y + self.sim_twist.linear.x*sampletime*math.sin(yaw_euler)
            updated_yaw = e[2] +self.sim_twist.angular.z*sampletime
        else:
            self.sim_pose.position.x = self.sim_pose.position.x + self.cmdvel_linear_x*sampletime*math.cos(yaw_euler)
            self.sim_pose.position.y = self.sim_pose.position.y + self.cmdvel_linear_x*sampletime*math.sin(yaw_euler)
            updated_yaw = e[2] + self.cmdvel_angular_z*sampletime

        updated_quaternion =tf.transformations.quaternion_from_euler(0, 0, updated_yaw)
        self.sim_pose.orientation.x = updated_quaternion[0]
        self.sim_pose.orientation.y = updated_quaternion[1]
        self.sim_pose.orientation.z = updated_quaternion[2]
        self.sim_pose.orientation.w = updated_quaternion[3]

        #for GUI
        out_str = ''
        vel_kmh = "{0:.3f}".format(self.sim_twist.linear.x * 3600.0 / 1000.0)
        yawrate = "{0:.3f}".format(self.sim_twist.angular.z)
        pose_x = "{0:.3f}".format(self.sim_pose.position.x)
        pose_y = "{0:.3f}".format(self.sim_pose.position.y)
        orientation = "{0:.3f}".format(self.sim_pose.orientation.z)
        out_str = out_str + "\n velocity[km/h]:" + vel_kmh
        out_str = out_str + "\n yawrate[rad/s]:" + yawrate
        out_str = out_str + "\n position[m]:(" + pose_x +"," + pose_y + ")"
        out_str = out_str + "\n orientation[rad]:" + orientation
        self.buffer.set(out_str)

        #update timestamp
        self.odom_header.seq =self.odom_header.seq + 1
        self.odom_header.stamp = rospy.Time.now()
        self.sim_odom.header = self.odom_header
        self.sim_odom.pose.pose = self.sim_pose
        self.sim_odom.twist.twist = self.sim_twist
        self.emu_odom_pub.publish(self.sim_odom)

        #update TF

        self.map_broadcaster.sendTransform(
            (self.sim_odom.pose.pose.position.x, self.sim_odom.pose.pose.position.y, self.sim_odom.pose.pose.position.z),
            updated_quaternion,
            rospy.Time.now(),
            "odom",
            "map"
        )

        base_link_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        self.odom_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            base_link_quat,
            rospy.Time.now(),
            # "base_link",
            "base_link",
            "odom"
        )

        #Save path_plan
        if self.save_path_as_csv == True:
            addRow = [0,self.sim_pose.position.x,self.sim_pose.position.y,0,updated_quaternion[0],updated_quaternion[1],updated_quaternion[2],updated_quaternion[3],
                      self.sim_twist.linear.x,self.sim_twist.linear.y,self.sim_twist.linear.z,0,0,self.sim_twist.angular.z]
            self.path_dict[len(self.path_dict)] = addRow
        self.root.after(100, self.update_odom)  #interval for gui update (100msec)

    ############
    # save csv #
    ############
    def save_csv(self):
        # Save CSV path file
        cols = ["time", "x", "y", "z", "w0", "w1", "w2", "w3", "vx", "vy", "vz", "roll", "pitch", "yaw"]
        df = pd.DataFrame.from_dict(self.path_dict, orient='index',columns=cols)
        df.to_csv("path_data.csv", index=False)

    #####################
    # cmdvel subscriber #
    #####################
    def cb_get_cmdvel_subscriber(self, msg):
        self.cmdvel_linear_x =msg.linear.x
        self.cmdvel_linear_y =msg.linear.x
        self.cmdvel_angular_z =msg.angular.z
        pass



    #######################
    # ctrl and c callabck #
    #######################
    def ctr_c_interruption(self, signum, frame):
        self.root.quit()
        self.root.update()
        self.save_csv()
        print("finish")

if __name__ == '__main__':
    print('Simple Odometry Simulator is Started...')
    test = Simple_odom_simulator()
    test.a.focus_set()
    test.update_odom()
    test.root.mainloop()
    test.save_csv()
    print("finish")
    pass
