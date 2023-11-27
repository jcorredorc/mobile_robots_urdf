/* 
@description:
This node does the following:
1.- It subscribes to the topics "/omniX/odom", with X=1,2, to obtain the posture and velocity of the robots.
2.- Taking into account the cinematic model of the omnidirectional robot composed of 4 mecanum wheels, the corresponding rotational velocity of each wheel is published in the "/omniX/wheelK_joint_vel_contr/command" topics, with X=1,2, K=1,...,4.
3.- Using the robots frames, "omniX/odom", the tf tree is setup to visualize correctly both mobile robots in RViz.
4.- Finally, the position of the center of each robot is published in the "omniX/path" topics, to display the robots path in RViz. 

Remark: If only one omni mobile robot is used, this node can be executed without any change.

@author: C. Mauricio Arteaga-Escamilla

-- To finish this node, please press ctrl+C.
DIRECTIONS:
In different shells:
$ roslaunch my_ugv_description omni_robot_4wheels.launch
	# Launchs one or two simulated omnidirectional robots in Gazebo using an empty world, loads the robots controllers, and opens RViz using my configuration file

$ rosrun my_ugv_description omni_tf_4wheels
	# Run this node
*/
#include "ros/ros.h"
#include "nav_msgs/Odometry.h" //Message type used by ".../odom" topic
#include "std_msgs/Float64.h"  //Message type used by ".../command" topic
#include "nav_msgs/Path.h" //Message type used by ".../path" topic
#include "tf/transform_broadcaster.h" //Needed since the tf class is used

using namespace std;

//Robot parameters
float Lx = 0.08, Ly = 0.12, R = 0.04; //Distances with respect to the robot frame and wheels radius

//Global objects and message instances for publishing and subscribing
ros::Publisher w1_r1_pub, w2_r1_pub, w3_r1_pub, w4_r1_pub;
ros::Publisher w1_r2_pub, w2_r2_pub, w3_r2_pub, w4_r2_pub;
std_msgs::Float64 w1_r1_msg, w2_r1_msg, w3_r1_msg, w4_r1_msg;
std_msgs::Float64 w1_r2_msg, w2_r2_msg, w3_r2_msg, w4_r2_msg;
ros::Subscriber pose_sub1, pose_sub2;
geometry_msgs::PoseStamped pose_1, pose_2;
ros::Publisher path_pub_1, path_pub_2;
nav_msgs::Path path_msg_1, path_msg_2;

//The "omni1/odom" frame is chosen as the global coordinate frame for ALL robots
const string the_frame = "omni1/odom"; //In OdomCallbackR2, the tree tf is setup, that is, omni2/odom = omni1/odom

void OdomCallbackR1(const nav_msgs::Odometry::ConstPtr & msg){//Function to get the robot1 velocities
  double vx, vy, wz; //Local variables

  pose_1.pose.position = msg->pose.pose.position;
  path_msg_1.poses.push_back(pose_1); //Add the robot position to the list of points

  //Get the robot velocities
  vx = msg->twist.twist.linear.x;
  vy = msg->twist.twist.linear.y;
  wz = msg->twist.twist.angular.z;
  //cout << "vx1: " << vx << "vy1: " << vy << "wz1: " << wz << endl;

  //Compute the velocity of each mecanum wheel
  w1_r1_msg.data = (vx-vy-(Lx+Ly)*wz)/R;
  w2_r1_msg.data = (vx+vy+(Lx+Ly)*wz)/R;
  w3_r1_msg.data = (vx-vy+(Lx+Ly)*wz)/R;
  w4_r1_msg.data = (vx+vy-(Lx+Ly)*wz)/R;
  
  //Compute the robot velocities from the wheels velocities
  /*double Vx, Vy, Wz, sum = Lx+Ly;
  Vx = (w1_r1_msg.data+w2_r1_msg.data+w3_r1_msg.data+w4_r1_msg.data)*R/4;
  Vy = (-w1_r1_msg.data+w2_r1_msg.data-w3_r1_msg.data+w4_r1_msg.data)*R/4;
  Wz = (-w1_r1_msg.data+w2_r1_msg.data+w3_r1_msg.data-w4_r1_msg.data)*R/(4*sum);
  cout << "Vx1: " << Vx << "Vy1: " << Vy << "Wz1: " << Wz << endl;*/
}

//Important: If the "/omni2/odom" topic exists, then this function will be executed
void OdomCallbackR2(const nav_msgs::Odometry::ConstPtr & msg){//This function is similar to the OdomCallbackR1 function
  double vx, vy, wz;

  pose_2.pose.position = msg->pose.pose.position;
  path_msg_2.poses.push_back(pose_2);

  vx = msg->twist.twist.linear.x;
  vy = msg->twist.twist.linear.y;
  wz = msg->twist.twist.angular.z;
  //cout << "vx2: " << vx << "vy2: " << vy << "wz2: " << wz << endl;

  w1_r2_msg.data = (vx-vy-(Lx+Ly)*wz)/R;
  w2_r2_msg.data = (vx+vy+(Lx+Ly)*wz)/R;
  w3_r2_msg.data = (vx-vy+(Lx+Ly)*wz)/R;
  w4_r2_msg.data = (vx+vy-(Lx+Ly)*wz)/R;


  //Important: To visualize correctly both mobile robots in RViz, this portion of code is required
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  //It is assumed that the "omni1/odom" and "omni2/odom" frames exist
  br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "omni1/odom", "omni2/odom") );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "omni_tf_4wheels"); //Node name
  ros::NodeHandle nh;
  ros::Rate loop_rate(50); //Node frequency (Hz)

  pose_sub1 = nh.subscribe("/omni1/odom", 1, OdomCallbackR1);
  pose_sub2 = nh.subscribe("/omni2/odom", 1, OdomCallbackR2);

  w1_r1_pub = nh.advertise<std_msgs::Float64>("/omni1/wheel1_joint_vel_contr/command", 1);
  w2_r1_pub = nh.advertise<std_msgs::Float64>("/omni1/wheel2_joint_vel_contr/command", 1);
  w3_r1_pub = nh.advertise<std_msgs::Float64>("/omni1/wheel3_joint_vel_contr/command", 1);
  w4_r1_pub = nh.advertise<std_msgs::Float64>("/omni1/wheel4_joint_vel_contr/command", 1);

  w1_r2_pub = nh.advertise<std_msgs::Float64>("/omni2/wheel1_joint_vel_contr/command", 1);
  w2_r2_pub = nh.advertise<std_msgs::Float64>("/omni2/wheel2_joint_vel_contr/command", 1);
  w3_r2_pub = nh.advertise<std_msgs::Float64>("/omni2/wheel3_joint_vel_contr/command", 1);
  w4_r2_pub = nh.advertise<std_msgs::Float64>("/omni2/wheel4_joint_vel_contr/command", 1);

  path_pub_1 = nh.advertise<nav_msgs::Path>("/omni1/path", 10);
  path_pub_2 = nh.advertise<nav_msgs::Path>("/omni2/path", 10);

  //Important: It is assigned the same reference frame for all mobile robots
  path_msg_1.header.frame_id = path_msg_2.header.frame_id = the_frame;

  ROS_WARN_ONCE("Check that Simulation is running.\nNode running...\n");
  printf("To finish this node, press ctrl+C\n");

  while (ros::ok()){
    //Publish the wheels velocities
    w1_r1_pub.publish(w1_r1_msg); w2_r1_pub.publish(w2_r1_msg);
    w3_r1_pub.publish(w3_r1_msg); w4_r1_pub.publish(w4_r1_msg);

    path_pub_1.publish(path_msg_1); //Publish the robot path

    w1_r2_pub.publish(w1_r2_msg); w2_r2_pub.publish(w2_r2_msg);
    w3_r2_pub.publish(w3_r2_msg); w4_r2_pub.publish(w4_r2_msg);

    path_pub_2.publish(path_msg_2);

    ros::spinOnce();    //Required for receiving callback functions
    loop_rate.sleep();  //Command to wait the rest of the time to complete the loop rate
  }

  printf("\n Node finished\n");
  return 0;
}

