/*
Name:        fake_ultrasound
Purpose:     published false ultrasonic values
Author:      Matthieu MAGNON
Created:     April 2018

Env : ROS kinetic
-------------------------------------------------------------------------------
List of Classes:
  

List of methodes:
 
 */

#include <iostream>
#include <stdlib.h>
#include <cstdlib>
#include <stdio.h>
#include "ros/ros.h"
#include "sensor_msgs/Range.h"

using namespace std;

int main(int argc, char *argv[]){  
    
    ROS_INFO("Starting Fake UltraSound node");

	/* node initialisation */
    ros::init(argc, argv, "ultrasound");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Publisher pub = nh.advertise<sensor_msgs::Range>("fake_ultrasound",10);
    ros::Rate loop_rate(100);

    sensor_msgs::Range us;

    us.radiation_type = 0;
    us.field_of_view = 1.4; //rad soit 80deg
    us.min_range = 0.0; // m
    us.max_range = 4.5; // m
    us.header.frame_id = "ultrasound";


    while (ros::ok()) {
    	int sigma = 100;
    	float rg = ( rand() % sigma + 1000 - sigma/2 )/1000; // distance en m
    	us.range = rg;
    	us.header.stamp = ros::Time::now();
    	pub.publish(us);
    	ros::spinOnce();
    	loop_rate.sleep();
    }


    return 0;
    
}