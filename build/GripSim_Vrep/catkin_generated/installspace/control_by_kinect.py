#!/usr/bin/env python2
import rospy
import std_msgs.msg as msg
import threading
import numpy as np
import sys
import os
from utils import *

robot_joints = np.array([math.pi/2,-math.pi/2,0,-math.pi/2,0,0])
robot_end_pose =  np.array([0,0,1,0,0,0])

def update_robot_joints(msg):
	global robot_joints
	robot_joints =  np.array(msg.data)

def update_robot_end_pose(msg):
       global robot_end_pose
       robot_end_pose = np.array(msg.data)

def find_point_center():
	data = np.loadtxt(open("/home/edifier/code/RoboSim_ws/src/GripSim_Vrep/scripts/output.csv","rb"),delimiter=",")
	P_center=np.array([np.mean(data[:,[0]]),np.mean(data[:,[1]]),np.mean(data[:,[2]])])
	data_x=-data[:,[0]]+0.0141
	data_y=data[:,[2]]+0.229
	data_z=data[:,[1]]+0.0824
	new_data = np.where((data_z >0.001) & (data_y <1))
	P_center[0]=np.mean(data_x[new_data])
	P_center[1]=np.mean(data_y[new_data])
	P_center[2]=np.mean(data_z[new_data])	
	return P_center

def main():
        # ROS
        rospy.init_node("ros_client", anonymous=True)
        pub = rospy.Publisher("/robot/target_joints", msg.Float64MultiArray, queue_size=10)
        joint_sub = rospy.Subscriber("/robot/joints", msg.Float64MultiArray, update_robot_joints)
        end_pose_sub = rospy.Subscriber("/robot/end_pose", msg.Float64MultiArray, update_robot_end_pose)

        P_center=find_point_center()
        target_pose = np.array([P_center[0],P_center[1],P_center[2],0,0,0])

        print(target_pose)
        # Controller config
        frequency = 10
        dt = 0.1
        ii=1.
        rate = rospy.Rate(frequency) 
        global robot_end_pose
        global robot_joints
        while not rospy.is_shutdown():
                ii+=1
                Jaco = Jacob(np.array(robot_joints))
                error = target_pose - robot_end_pose
                w_weight=np.array([[0.5,0.,0.,0.,0.,0.],[0.,0.5,0.,0.,0.,0.],[0.,0.,0.5,0.,0.,0.],[0.,0.,0.,.01,0.,0.],[0.,0.,0.,0.,.01,0.],[0.,0.,0.,0.,0.,.01]])
                d_pose = np.matmul(np.linalg.pinv(Jaco) ,dt*np.matmul(w_weight,error))
                if abs( np.linalg.det(Jaco) )<0.01:
                        d_pose = np.array([-0.01,-0.01,-0.01,0.01,0.01,0.01])

                new_joints = np.array(d_pose) + robot_joints
                new_msg = msg.Float64MultiArray()  
                new_msg.data = new_joints.tolist()
                print(new_joints)
                pub.publish(new_msg)    
                rate.sleep()


if __name__ == "__main__":
    main()

