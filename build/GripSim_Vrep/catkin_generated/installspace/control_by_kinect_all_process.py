#!/usr/bin/env python2
import rospy
import std_msgs.msg as msg
from  sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import time
import numpy as np
from utils import *

robot_joints = None
robot_end_pose =  None
point_cloud = None
depth_camera_frame_in_world = np.array([
    [-1.00, 0, 0.0, -0.01410000026226],
    [0.0, 0, 1.0, 0.22925010323524],
    [0, 1.00, 0, 0.0823894739151]])

def update_robot_joints(msg):
        global robot_joints
        robot_joints =  np.array(msg.data)

def update_robot_end_pose(msg):
       global robot_end_pose
       robot_end_pose = np.array(msg.data)

def update_point_clouds(msg):
       global point_cloud
       point_cloud = point_cloud2.read_points_list(msg, field_names= ("x", "y", "z"))

def find_point_center():
        global point_cloud
        global depth_camera_frame_in_world
        x_sum ,y_sum ,z_sum,count = 0,0,0,0
        for point in point_cloud:
                if point.y > 0.001 and point.z < 1:
                        count += 1
                        x_sum += point.x
                        y_sum += point.y
                        z_sum += point.z
        P_center_in_camera=np.array([x_sum/count, y_sum/count, z_sum/count, 1]).reshape((4,1))
        P_center_in_world = np.matmul(depth_camera_frame_in_world, P_center_in_camera)
        P_center = np.array([P_center_in_world[0,0], P_center_in_world[1,0], P_center_in_world[2,0]])
        return P_center

def main():
       # ROS
       rospy.init_node("ros_client", anonymous=True)
       pub = rospy.Publisher("/robot/target_joints", msg.Float64MultiArray, queue_size=10)
       joint_sub = rospy.Subscriber("/robot/joints", msg.Float64MultiArray, update_robot_joints)
       end_pose_sub = rospy.Subscriber("/robot/end_pose", msg.Float64MultiArray, update_robot_end_pose)
       rospy.Subscriber("/kinect/depth", PointCloud2, update_point_clouds)
       # STEP1: find target pose
       while not point_cloud:
              print("Receiving point clouds data ...")
              time.sleep(1)
       P_center=find_point_center()
       target_pose = np.array([P_center[0],P_center[1],P_center[2],0,0,0])
       print("Target pose is {}".format(target_pose))
       # STEP2: control robot 
       frequency = 10
       dt = 1. / frequency
       rate = rospy.Rate(frequency) 
       global robot_end_pose
       global robot_joints
       while not rospy.is_shutdown():
                this_robot_joints = robot_joints
                Jaco = Jacob(np.array(this_robot_joints))
                error = target_pose - robot_end_pose
                w_weight=np.array([[0.5,0.,0.,0.,0.,0.],[0.,0.5,0.,0.,0.,0.],[0.,0.,0.5,0.,0.,0.],[0.,0.,0.,.01,0.,0.],[0.,0.,0.,0.,.01,0.],[0.,0.,0.,0.,0.,.01]])
                d_pose = np.matmul(np.linalg.pinv(Jaco) ,dt*np.matmul(w_weight,error))
                if abs( np.linalg.det(Jaco) )<0.01:
                       d_pose = np.array([-0.01,-0.01,-0.01,0.01,0.01,0.01])
                #print d_pose
                new_joints = np.array(d_pose) + this_robot_joints
                new_msg = msg.Float64MultiArray()  
                new_msg.data = new_joints.tolist()
                pub.publish(new_msg)    
                rate.sleep()


if __name__ == "__main__":
    main()

