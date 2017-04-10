#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <time.h>

#include "../../../../../home/pulver/drl_ws/install/include/deep_reinforced_landing/ResetPosition.h"
#include "../../../../../home/pulver/drl_ws/install/include/deep_reinforced_landing/SendCommand.h"

uint32_t state;
geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_hover;
std_msgs::Empty emp_msg;

using namespace std;

void getStateCallBack(const ardrone_autonomy::NavdataConstPtr &msg);

int main(int argc, char **argv)
{
  ROS_INFO("Flying ARdrone");
  ros::init(argc, argv, "ARDrone_test");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);
  ros::Publisher pub_empty_take_off;
  ros::Publisher pub_empty_land;
  ros::Publisher pub_twist;
  ros::Publisher pub_empty_reset;
  ros::Subscriber sub_drone_state;
  pub_empty_reset = node.advertise<std_msgs::Empty>("/quadrotor/ardrone/reset", 1);
  pub_twist = node.advertise<geometry_msgs::Twist>("/quadrotor/cmd_vel", 1);
  pub_empty_land = node.advertise<std_msgs::Empty>("/quadrotor/ardrone/land", 1);
  pub_empty_take_off = node.advertise<std_msgs::Empty>("/quadrotor/ardrone/takeoff", 1);
  sub_drone_state = node.subscribe<ardrone_autonomy::Navdata>("/quadrotor/ardrone/navdata", 1, getStateCallBack);

  ros::ServiceClient reset_pose_service;
  ros::ServiceClient send_command_service;
  reset_pose_service = node.serviceClient<deep_reinforced_landing::ResetPosition>("/drl/set_model_state");
  send_command_service = node.serviceClient<deep_reinforced_landing::SendCommand>("/drl/send_command");
  deep_reinforced_landing::ResetPosition srv_reset;
  deep_reinforced_landing::SendCommand srv_command;

  string commands[] = {"left", "right", "forward", "backward", "ascend","descend", "rotate_left", "rotate_right"};

  // hover message
  twist_msg_hover.linear.x = 0.0;
  twist_msg_hover.linear.y = 0.0;
  twist_msg_hover.linear.z = 0.0;
  twist_msg_hover.angular.x = 0.0;
  twist_msg_hover.angular.y = 0.0;
  twist_msg_hover.angular.z = 0.0;

  int i = 0; //number of flight

  while (ros::ok() && i<5)
  {
    // Generate 5 random flight

    ROS_INFO("------------");
    ROS_INFO("Iteration %d", i);
    int starting_time = ( int ) ros::Time::now().toSec();      // every flight takes 10 seconds
    while ( ros::Time::now().toSec() - starting_time <= 10)
    {
      switch (state)
      {
      case (2):                              /* if the drone is landed on the ground*/
        pub_empty_take_off.publish(emp_msg); /* launches the drone */
        ros::Duration(1.0).sleep();
        ROS_INFO("Command: takeoff");
        break;

        //      case (3): /*if the drone is flying*/
        //        /* Move along x axis */
        //        twist_msg.linear.x = 1.0;
        //        twist_msg.linear.y = 0.0;
        //        twist_msg.linear.z = 0.0;
        //        twist_msg.angular.x = 0.0;
        //        twist_msg.angular.y = 0.0;
        //        twist_msg.angular.z = 0.0;
        //        pub_twist.publish(twist_msg);
        //        sleep(3);

        //        /* Hover */
        //        pub_twist.publish(twist_msg_hover);
        //        sleep(3);

        //        /* Move along y */
        //        twist_msg.linear.x = 0.0;
        //        twist_msg.linear.y = 0.3;
        //        twist_msg.linear.z = 0.0;
        //        twist_msg.angular.x = 0.0;
        //        twist_msg.angular.y = 0.0;
        //        twist_msg.angular.z = 0.0;
        //        pub_twist.publish(twist_msg);
        //        sleep(3);

        //        /* Hover */
        //        pub_twist.publish(twist_msg_hover);
        //        sleep(3);

        //        /* Rotate along z */
        //        twist_msg.linear.x = 0.0;
        //        twist_msg.linear.y = 0.0;
        //        twist_msg.linear.z = 0.0;
        //        twist_msg.angular.x = 0.0;
        //        twist_msg.angular.y = 0.0;
        //        twist_msg.angular.z = 0.4;
        //        pub_twist.publish(twist_msg);
        //        sleep(4);

        //        /* Hover */
        //        pub_twist.publish(twist_msg_hover);
        //        sleep(3);

        //        /* Land */
        //        pub_empty_land.publish(emp_msg);
        //        exit(0);
      default:
        // first stop the UAV from the previous command
        srv_command.request.command = "stop";
        if(send_command_service.call(srv_command))
        {
          // wait for one second
          ros::Duration(0.1).sleep();
        }
        else
        {
          ROS_ERROR("Stop service has not been called");
        }

        // then choose a new command
        int random_command = rand() % (sizeof(commands)/sizeof(*commands));
        srv_command.request.command = commands[random_command];
        if(send_command_service.call(srv_command))
        {
          // wait for one second
          ROS_INFO("Command: %s", commands[random_command].c_str());
          ros::Duration(0.9).sleep();
        }
        else
        {
          ROS_ERROR("Command service has not been called");
        }
      }
    }
    srv_reset.request.reset = true;
    if(reset_pose_service.call(srv_reset))
    {
      // wait for one second
      ROS_INFO("POSITION RESETTED");
      //sleep(1);
    }
    else
    {
      ROS_ERROR("Reset service has not been called!!");
    }

    i++;
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;

}

void getStateCallBack(const ardrone_autonomy::NavdataConstPtr &msg)
{
  state = msg->state;

  switch (state)
  {
  case (2):
    ROS_INFO("Landed");
    break;

  case (3):
    ROS_INFO("Flying");
    break;

  case (4):
    ROS_INFO("Hovering");
    break;

  case (6):
    ROS_INFO("Taking off");
    break;

  case (8):
    ROS_INFO("Landing");
    break;
  }
}
