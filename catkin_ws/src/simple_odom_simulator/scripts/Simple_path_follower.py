#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import math
# import for ros function
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path, Odometry

#######################################
# Simple Path follower (Pure Pursuit) #
#######################################
class Simple_path_follower():

    ##################
    # Initialization #
    ##################
    def __init__(self):

        rospy.init_node('Simple_Path_Follower', anonymous=True)
        self.r = rospy.Rate(50)  # 50hz

        self.target_speed = 1.0             #target speed [km/h]
        self.target_LookahedDist = 0.5      #Lookahed distance for Pure Pursuit[m]

        #first flg (for subscribe global path topic)
        self.path_first_flg = False
        self.odom_first_flg = False
        self.position_search_flg = False
        self.last_indx = 0

        #initialize publisher
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50)
        self.lookahed_pub = rospy.Publisher("/lookahed_marker", Marker, queue_size=50)

        #initialize subscriber
        self.path_sub = rospy.Subscriber("/path", Path, self.cb_get_path_topic_subscriber)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.cb_get_odometry_subscriber)



    def publish_lookahed_marker(self,x,y,yaw_euler):

        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = "my_name_space"
        marker_data.id = 0

        marker_data.action = Marker.ADD

        marker_data.pose.position.x = x
        marker_data.pose.position.y = y
        marker_data.pose.position.z = 0.0

        temp_quaternion = tf.transformations.quaternion_from_euler(0,0,yaw_euler)

        marker_data.pose.orientation.x = temp_quaternion[0]
        marker_data.pose.orientation.y = temp_quaternion[1]
        marker_data.pose.orientation.z = temp_quaternion[2]
        marker_data.pose.orientation.w = temp_quaternion[3]

        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.1
        marker_data.scale.y = 0.1
        marker_data.scale.z = 0.1

        marker_data.lifetime = rospy.Duration()
        marker_data.type = 0

        self.lookahed_pub.publish(marker_data)


    ###################
    # Update cmd_vel  #
    ###################
    def update_cmd_vel(self):
        if self.path_first_flg == True and self.odom_first_flg == True:

            dist_from_current_pos_np = np.sqrt(np.power((self.path_x_np-self.current_x),2) + np.power((self.path_y_np-self.current_y),2))
            min_indx = dist_from_current_pos_np.argmin()
            nearest_x = self.path_x_np[min_indx]
            nearest_y = self.path_y_np[min_indx]
            # Get nearest Path point at first time
            if self.position_search_flg == False:
                self.pass_flg_np[0:min_indx] = 1    #Set pass flg
                self.position_search_flg = True
            else:
                # Check pass flg from vehicle position
                for indx in range (self.last_indx,self.path_x_np.shape[0]):
                    if dist_from_current_pos_np[indx] < 1.0:
                        self.pass_flg_np[indx] = 1
                    else:
                        break
            self.last_indx = min_indx

            #check goal
            if self.pass_flg_np[self.path_x_np.shape[0]-1] == 1:
                cmd_vel = Twist()
                self.cmdvel_pub.publish(cmd_vel)
                print("goal!!")
                return
            #calculate target point
            dist_sp_from_nearest = 0.0
            target_lookahed_x = nearest_x
            target_lookahed_y = nearest_y
            for indx in range (self.last_indx,self.path_x_np.shape[0]):
                dist_sp_from_nearest = self.path_st_np[indx] - self.path_st_np[self.last_indx]
                if (dist_sp_from_nearest) > self.target_LookahedDist:
                    target_lookahed_x = self.path_x_np[indx]
                    target_lookahed_y = self.path_y_np[indx]
                    break

            #calculate target yaw rate
            target_yaw = math.atan2(target_lookahed_y-self.current_y,target_lookahed_x-self.current_x)

            #check vehicle orientation
            # if target_yaw - self.target_yaw_last < -math.pi:
            #     target_yaw = 2*math.pi + target_yaw
            # elif target_yaw - self.target_yaw_last > math.pi:
            #     target_yaw = 2*math.pi - target_yaw

            yaw_diff = target_yaw - self.current_yaw_euler

            if yaw_diff > math.pi:
                yaw_diff = yaw_diff % math.pi
            elif yaw_diff < -math.pi:
                yaw_diff = yaw_diff%(-math.pi)


            sample_sec = dist_sp_from_nearest/(self.target_speed/3.6)
            if sample_sec != 0.0:
                yaw_rate = math.fabs(yaw_diff)/sample_sec
            else:
                yaw_rate = 0.0

            # check vehicle orientation and target yaw
            if math.fabs(target_yaw - self.current_yaw_euler) < math.pi:
                if (target_yaw) < (self.current_yaw_euler):
                    yaw_rate = yaw_rate * (-1.0)
            elif math.fabs(target_yaw - self.current_yaw_euler) > math.pi:
                if (target_yaw) > (self.current_yaw_euler):
                    yaw_rate = yaw_rate * (-1.0)

            print(yaw_diff*180/math.pi,target_yaw*180/math.pi,self.current_yaw_euler*180/math.pi)

            #Set Cmdvel
            cmd_vel = Twist()
            cmd_vel.linear.x = self.target_speed/3.6    #[m/s]
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = yaw_rate
            self.cmdvel_pub.publish(cmd_vel)

            #publish maker
            self.publish_lookahed_marker(target_lookahed_x,target_lookahed_y,target_yaw)
            #print("cmd_vel_update")
            self.r.sleep()
            return


    ####################################
    # Callback for receiving Odometry  #
    ####################################
    def cb_get_odometry_subscriber(self,msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        e = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        yaw_euler = e[2]
        self.current_yaw_euler = yaw_euler
        self.odom_first_flg = True
        #print("odom_sub")

    ######################################
    # Callback for receiving path topic  #
    ######################################
    def cb_get_path_topic_subscriber(self,msg):
        if self.path_first_flg != True:
            self.path_x_np = np.zeros([len(msg.poses)])
            self.path_y_np = np.zeros([len(msg.poses)])
            self.path_st_np = np.zeros([len(msg.poses)])
            self.pass_flg_np = np.zeros([len(msg.poses)])
            last_x = 0.0
            last_y = 0.0
            last_st = 0.0
            for indx in range(len(msg.poses)):
                self.path_x_np[indx] = msg.poses[indx].pose.position.x
                self.path_y_np[indx] = msg.poses[indx].pose.position.y
                self.path_st_np[indx] = last_st + math.sqrt((self.path_x_np[indx]-last_x)**2 + (self.path_y_np[indx]-last_y)**2)
                last_x = self.path_x_np[indx]
                last_y = self.path_y_np[indx]
                last_st = self.path_st_np[indx]
            self.path_first_flg = True
            #print("path(first)")

if __name__ == '__main__':
    print('Path following is Started...')
    test = Simple_path_follower()
    try:
        while not rospy.is_shutdown():
            test.update_cmd_vel()
    except KeyboardInterrupt:
        print("finished!")


