#include "unistd.h"
#include "stdio.h"
#include "string.h"
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"
using namespace std;
ros::Publisher o_publisher;
ros::Publisher p_publisher;
ros::Publisher b_publisher;
ros::Publisher t_publisher;
double fakeVals[8] = { 0 };



void pos(const ros::TimerEvent&) {/* temp timer callback *//* Send encoded message  */
//	ROS_WARN("Hoping to publish pos %f, %f, %f", fakeVals[0], fakeVals[1], fakeVals[2]);
	geometry_msgs::Vector3Stamped tmp;
	tmp.vector.x = fakeVals[0];
	tmp.vector.y = fakeVals[1];
	tmp.vector.z = fakeVals[2];
//	ROS_WARN("Actually publishing pos %f, %f, %f", tmp.vector.x, tmp.vector.y, tmp.vector.y);
	p_publisher.publish(tmp);
}
void ori(const ros::TimerEvent&) {/* temp timer callback *//* Send encoded message  */
	geometry_msgs::Vector3Stamped tmp;
	tmp.vector.x = fakeVals[3];
	tmp.vector.y = fakeVals[4];
	tmp.vector.z = fakeVals[5];
	o_publisher.publish(tmp);
}
void bat(const ros::TimerEvent&) {/* temp timer callback *//* Send encoded message  */
	std_msgs::Float32 tmp;
	tmp.data = fakeVals[6];
	b_publisher.publish(tmp);
}
void tim(const ros::TimerEvent&) {/* temp timer callback *//* Send encoded message  */
	for(int x = 0; x < 8; x++){
				fakeVals[x] += x*0.1;
				if (fakeVals[x] > 50)
					fakeVals[x] = 0;

			}
	std_msgs::Float32 tmp;
	tmp.data = fakeVals[7];
	t_publisher.publish(tmp);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "xbee_node");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	ROS_WARN("In fakePublisher");
	std::string p_orientation_topic;
	std::string p_position_topic;
	std::string p_battery_topic;
	std::string p_time_topic;

	n.param<std::string>("p_orientation_topic", p_orientation_topic,"/fakeOrientation");
	n.param<std::string>("p_position_topic", p_position_topic, "/fakePosition");
	n.param<std::string>("p_battery_topic", p_battery_topic, "/fakeBattery");
	n.param<std::string>("p_time_topic", p_time_topic, "/fakeTime");

	o_publisher = nh.advertise<geometry_msgs::Vector3Stamped>(p_orientation_topic.c_str(), 20, 1);
	p_publisher = nh.advertise<geometry_msgs::Vector3Stamped>(p_position_topic.c_str(), 20, 1);
	b_publisher = nh.advertise<std_msgs::Float32>(p_battery_topic.c_str(), 20, 1);
	t_publisher = nh.advertise<std_msgs::Float32>(p_time_topic.c_str(), 20, 1);

	ros::Timer timer1 = nh.createTimer(ros::Duration(0.1), pos);
	ros::Timer timer2 = nh.createTimer(ros::Duration(0.2), ori);
	ros::Timer timer3 = nh.createTimer(ros::Duration(1), bat);
	ros::Timer timer4 = nh.createTimer(ros::Duration(2), tim);

	ros::spin();
	return 0;
}
