#!/usr/bin/env python3

#By default, use python3 for ROS Noetic, and python for ROS Kinetic and Melodic
"""This python script is the equivalent to the 'src/omni_tf_4wheels.cpp', so it executes the same tasks.
For more information, please refer to that source file.

-- To finish this node, please press 'ctrl+C'."""

__author__ = "C. Mauricio Arteaga-Escamilla"

import rospy, tf
from nav_msgs.msg import Odometry, Path #Message type to subscribe to /odom and to publish in /path topics 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64 #Message type to publish velocities to the wheels
from geometry_msgs.msg import PoseStamped #Message type to publish the robots' path
#from math import sin, cos, pi

#Robot parameters. Distances with respect to the robot frame and wheels radius
Lx = 0.08
Ly = 0.12
R = 0.04

#Create the message instances
w1_r1_msg = Float64(); w2_r1_msg = Float64(); w3_r1_msg = Float64(); w4_r1_msg = Float64()
w1_r2_msg = Float64(); w2_r2_msg = Float64(); w3_r2_msg = Float64(); w4_r2_msg = Float64()
path_msg_1 = Path()
path_msg_2 = Path()

the_frame = "omni1/odom"; #In OdomCallbackR2, the tree tf is setup, that is, omni2/odom = omni1/odom


def OdomCallbackR1(msg): #Callback function to get the robot1 velocities
	global w1_r1_msg, w2_r1_msg, w3_r1_msg, w4_r1_msg
	pose_1 = PoseStamped()
	pose_1.pose.position = msg.pose.pose.position
	path_msg_1.poses.append(pose_1) #Add the robot position to the list of points
        
	#Get the robot velocities
	vx = msg.twist.twist.linear.x
	vy = msg.twist.twist.linear.y
	wz = msg.twist.twist.angular.z
	#rospy.loginfo("vx1: %.2f, vy1: %.2f, wz1: %.2f\n", vx, vy, wz)
	
	#Compute the velocity of each mecanum wheel
	w1_r1_msg.data = (vx-vy-(Lx+Ly)*wz)/R
	w2_r1_msg.data = (vx+vy+(Lx+Ly)*wz)/R
	w3_r1_msg.data = (vx-vy+(Lx+Ly)*wz)/R
	w4_r1_msg.data = (vx+vy-(Lx+Ly)*wz)/R


#Important: If the "/omni2/odom" topic exists, then this function will be executed
def OdomCallbackR2(msg): #Callback function to get the robot 2 velocities
	global w1_r2_msg, w2_r2_msg, w3_r2_msg, w4_r2_msg
	pose_2 = PoseStamped()
	pose_2.pose.position = msg.pose.pose.position
	path_msg_2.poses.append(pose_2)
        
	#Get the robot velocities
	vx = msg.twist.twist.linear.x
	vy = msg.twist.twist.linear.y
	wz = msg.twist.twist.angular.z
	#rospy.loginfo("vx2: %.2f, vy2: %.2f, wz2: %.2f\n", vx, vy, wz)
	
	#Compute the velocity of each mecanum wheel
	w1_r2_msg.data = (vx-vy-(Lx+Ly)*wz)/R
	w2_r2_msg.data = (vx+vy+(Lx+Ly)*wz)/R
	w3_r2_msg.data = (vx-vy+(Lx+Ly)*wz)/R
	w4_r2_msg.data = (vx+vy-(Lx+Ly)*wz)/R
	
	
	#Important: Configuration of the tf tree in order to visualize correctly both mobile robots in RViz
	#It is assumed that the "omni1/odom" and "omni2/odom" frames exist
	br = tf.TransformBroadcaster()
	br.sendTransform( (0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "omni1/odom", "omni2/odom" )


def main_function():
	rospy.init_node('omni_tf_4wheels', anonymous=True) #Initialize the node. anonymous=True for multiple nodes
	rate = rospy.Rate(50) #Node frequency (Hz)
	
	rospy.Subscriber('/omni1/odom',Odometry, OdomCallbackR1) #To subscribe to the topic
	rospy.Subscriber('/omni2/odom',Odometry, OdomCallbackR2)

	w1_r1_pub = rospy.Publisher('/omni1/wheel1_joint_vel_contr/command', Float64, queue_size=10)
	w2_r1_pub = rospy.Publisher('/omni1/wheel2_joint_vel_contr/command', Float64, queue_size=10)
	w3_r1_pub = rospy.Publisher('/omni1/wheel3_joint_vel_contr/command', Float64, queue_size=10)
	w4_r1_pub = rospy.Publisher('/omni1/wheel4_joint_vel_contr/command', Float64, queue_size=10)

	w1_r2_pub = rospy.Publisher('/omni2/wheel1_joint_vel_contr/command', Float64, queue_size=10)
	w2_r2_pub = rospy.Publisher('/omni2/wheel2_joint_vel_contr/command', Float64, queue_size=10)
	w3_r2_pub = rospy.Publisher('/omni2/wheel3_joint_vel_contr/command', Float64, queue_size=10)
	w4_r2_pub = rospy.Publisher('/omni2/wheel4_joint_vel_contr/command', Float64, queue_size=10)

	path_pub_1 = rospy.Publisher('/omni1/path', Path, queue_size=10)
	path_pub_2 = rospy.Publisher('/omni2/path', Path, queue_size=10)

	global path_msg_1, path_msg_2
	#Important: It is assigned the same reference frame for all mobile robots
	path_msg_1.header.frame_id = the_frame
	path_msg_2.header.frame_id = the_frame

	rospy.logwarn("Check that Simulation is running.\nNode running...\n")
	print("To finish this node, press ctrl+C\n")
	
	while not rospy.is_shutdown():
		#Publish the wheels velocities
		w1_r1_pub.publish(w1_r1_msg); w2_r1_pub.publish(w2_r1_msg)
		w3_r1_pub.publish(w3_r1_msg); w4_r1_pub.publish(w4_r1_msg)
		
		w1_r2_pub.publish(w1_r2_msg); w2_r2_pub.publish(w2_r2_msg)
		w3_r2_pub.publish(w3_r2_msg); w4_r2_pub.publish(w4_r2_msg)
		
		##Publish the robots path
		path_pub_1.publish(path_msg_1)
		path_pub_2.publish(path_msg_2)

		rate.sleep() #spinOnce() function does not exist in python
	
	print("\n Node finished\n")


if __name__ == '__main__':
    try:
        main_function() #Execute the function
    except rospy.ROSInterruptException:
        pass
