/*
   Parker Conroy
   Algorithmic Robotics Lab @ University of Utah

   This program launches the AR Drone.
   It is intended as a simple example for those starting with the AR Drone platform.
 */
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

uint32_t state;
geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_hover;
std_msgs::Empty emp_msg;

using namespace std;

void getStateCallBack (const ardrone_autonomy::NavdataConstPtr &msg);


int main (int argc, char **argv)
{
	ROS_INFO ("Flying ARdrone");
	ros::init (argc, argv, "ARDrone_test");
	ros::NodeHandle node;
	ros::Rate loop_rate (50);
	ros::Publisher pub_empty_take_off;
	ros::Publisher pub_empty_land;
	ros::Publisher pub_twist;
	ros::Publisher pub_empty_reset;
	ros::Subscriber sub_drone_state;
	pub_empty_reset = node.advertise<std_msgs::Empty> ("/ardrone/reset", 1);
	pub_twist = node.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
	pub_empty_land = node.advertise<std_msgs::Empty> ("/ardrone/land", 1);
	pub_empty_take_off = node.advertise<std_msgs::Empty> ("/ardrone/takeoff", 1);
	sub_drone_state = node.subscribe<ardrone_autonomy::Navdata> ("/ardrone/navdata", 1, getStateCallBack);

	
	//hover message
	twist_msg_hover.linear.x=0.0; 
	twist_msg_hover.linear.y=0.0;
	twist_msg_hover.linear.z=0.0;
	twist_msg_hover.angular.x=0.0; 
	twist_msg_hover.angular.y=0.0;
	twist_msg_hover.angular.z=0.0;  


	while (ros::ok()) {
			switch (state) {
				case (2) : /* if the drone is landed on the ground*/
					pub_empty_take_off.publish (emp_msg); /* launches the drone */
					sleep (2);
					break;


				case (3) : /*if the drone is flying*/
					/* Move along x axis */
					twist_msg.linear.x=1.0;
					twist_msg.linear.y=0.0;
					twist_msg.linear.z=0.0;
					twist_msg.angular.x=0.0;
					twist_msg.angular.y=0.0;
					twist_msg.angular.z=0.0;
					pub_twist.publish (twist_msg); 
					sleep (3);
					
					/* Hover */
					pub_twist.publish (twist_msg_hover); 
					sleep (3);
					
					/* Move along y */
					twist_msg.linear.x=0.0;
					twist_msg.linear.y=0.3;
					twist_msg.linear.z=0.0;
					twist_msg.angular.x=0.0;
					twist_msg.angular.y=0.0;
					twist_msg.angular.z=0.0;
					pub_twist.publish (twist_msg); 
					sleep (3);
					
					/* Hover */
					pub_twist.publish (twist_msg_hover); 
					sleep (3);
					
					/* Rotate along z */
					twist_msg.linear.x=0.0;
					twist_msg.linear.y=0.0;
					twist_msg.linear.z=0.0;
					twist_msg.angular.x=0.0;
					twist_msg.angular.y=0.0;
					twist_msg.angular.z=0.4;
					pub_twist.publish (twist_msg); 
					sleep (4);
					
					/* Hover */
					pub_twist.publish (twist_msg_hover); 
					sleep (3);
					
					/* Land */
					pub_empty_land.publish (emp_msg); 
					exit (0);
				}

			ros::spinOnce();
			loop_rate.sleep();
		}
}


void getStateCallBack (const ardrone_autonomy::NavdataConstPtr &msg)
{
	state = msg->state;

	switch (state) {
		case (2) :
			ROS_INFO ("Landed");
			break;

		case (3) :
			ROS_INFO ("Flying");
			break;

		case (4) :
			ROS_INFO ("Hovering");
			break;

		case (6) :
			ROS_INFO ("Taking off");
			break;

		case (8) :
			ROS_INFO ("Landing");
			break;

		}

}
