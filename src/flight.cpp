/*
   Parker Conroy
   Algorithmic Robotics Lab @ University of Utah

   This program launches the AR Drone.
   It is intended as a simple example for those starting with the AR Drone platform.
 */
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <iostream>
std_msgs::Empty emp_msg;

using namespace std;

void getStateCallBack (const ardrone_autonomy::NavdataConstPtr &msg);
uint32_t state;

int main (int argc, char **argv)
{
	ROS_INFO ("Flying ARdrone");
	ros::init (argc, argv, "ARDrone_test");
	ros::NodeHandle node;
	ros::Rate loop_rate (50);
	ros::Publisher pub_take_off, pub_land;
	ros::Subscriber sub_drone_state;
	pub_land = node.advertise<std_msgs::Empty> ("/ardrone/land", 1);  /* Message queue length is just 1 */
	pub_take_off = node.advertise<std_msgs::Empty> ("/ardrone/takeoff", 1);  /* Message queue length is just 1 */
	sub_drone_state = node.subscribe<ardrone_autonomy::Navdata> ("/ardrone/navdata", 1, getStateCallBack);
	double time_start, offset;


	while (ros::ok()) {
			/*
			pub_empty.publish (emp_msg);   // launches the drone
			ros::spinOnce();
			loop_rate.sleep();
			time_start = (double) ros::Time::now().toSec();
			offset = (double) ros::Duration (2,0).toSec();

			//ROS_INFO ("ARdrone launched");

			cout << "Tempo attuale: "<< (double) ros::Time::now().toSec() << endl;
			cout << "Tempo finale programato: "<< time_start << " + "<< offset << " = " << time_start + offset << endl;

			if ( (double) ros::Time::now().toSec() >= time_start + offset) {
			        //ros::shutdown();
			        cout << "spegni" <<endl;
			    }

			*/

			switch (state) {
				case (2) :
					pub_take_off.publish (emp_msg); /* launches the drone */
					break;
				
				
				case(3):
					sleep(5);
					pub_land.publish<>(emp_msg);
					exit(0);
			}	
			ros::spinOnce();
			loop_rate.sleep();
		} //ros::ok loop
}//main

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
			ROS_INFO ("Hoovering");
			break;

		case (6) :
			ROS_INFO ("Taking off");
			break;

		case (8) :
			ROS_INFO ("Landing");
			break;

		}

}
