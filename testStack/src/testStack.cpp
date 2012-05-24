/*
 * testStack.cpp
 *
 *  Created on: Feb 19, 2012
 *      Author: peter
 */

#include <stdio.h>
#include <ros/ros.h>
#include <fmMsgs/airframeState.h>

FILE* file;

void savetofile(const fmMsgs::airframeState::ConstPtr& msg) {
	char buf[256];

	sprintf(buf, "%5.2f %5.2f %5.2f", msg->header.stamp.toSec(), msg->pose.x, msg->pose.y);
	ROS_INFO("writing \"%s\" to file", buf);
	int len = sprintf(buf, "%s\n", buf);
	fwrite (buf, 1, len,file);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "grapper");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	file = fopen("/home/peter/master/workspace/FroboMind/testStack/file.txt", "wb");
	ros::Subscriber stateSubscriber = nh.subscribe("/airframeState", 1, savetofile);
	ros::spin();
	fclose(file);
}
