/*
 void gpsFixCallback (const gps_common::GPSFix::ConstPtr & msg) -> OSM_IS_GPS_MAP
 latitude
 longitude


 void llStatusCallback (const asctec_msgs::LLStatusConstPtr & dat) -> IS_GTK_GAUGE
 llStatus_.battery_voltage_1


 void imuCalcDataCallback (const asctec_msgs::IMUCalcDataConstPtr & dat) -> IS_GTK_VARIOMETER and IS_GTK_COMPASS
 imuCalcData_.angle_yaw
 imuCalcData_.angle_nick
 imuCalcData_.angle_roll
 imuCalcData_.dheight
 imuCalcData_.mag_heading

 void heightCallback (const mav_msgs::Height::ConstPtr& height) -> IS_GTK_ALTIMETER
 heightData_.climb
 heightData_.height

 void imuCallback (const sensor_msgs::ImuConstPtr & imu) -> IS_GTK_COMPASS and IS_GTK_ARTIFICIAL_HORIZON
 imuData_.orientation.x
 imuData_.orientation.y
 imuData_.orientation.z
 imuData_.orientation.w
 */

#include "ros/ros.h"
#include "fmMsgs/teleGround2Air.h"
#include "fmMsgs/teleAir2Ground.h"
#include "fmMsgs/serial.h"
#include <asctec_msgs/IMUCalcData.h>
#include <asctec_msgs/LLStatus.h>
#include <mav_msgs/Height.h>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/Imu.h>

const std::string imuTopic = "/mav/imu";
const std::string heightTopic = "/mav/pressure_height_filtered";
const std::string imuCalcDataTopic = "/asctec/IMU_CALCDATA";
const std::string gpsDataTopic = "/fix";
const std::string llStatusTopic = "/asctec/LL_STATUS";

ros::Publisher gps_publisher;
ros::Publisher bat_publisher;
ros::Publisher var_comp_publisher;
ros::Publisher alt_publisher;
ros::Publisher comp_arth_publisher;

fmMsgs::teleAir2Ground air2ground;
int flip = 0;
double fakebat = 0;
double theta = 0;

void xbeeCallback(const fmMsgs::teleAir2Ground::ConstPtr& msg) {
//Handle input message
	air2ground = (*msg);

//	Header header
//	fmVector3 orientation	# Orientation vector (ϕ, θ, ψ)
//	fmVector3 position	# Position vectos (Pn, Pe, Alt)
//	float64 airspeed	# Speed through air (Pitot measure) [m/s]
//	float64 truespeed	# True airspeed (wind compendsated) [m/s]
//	float64 groundspeed	# Speed over ground [m/s]
//	float64 battery		# Battery voltage [mV] - ALT: State of Charge [%]
//	float64 cpuload		# Load on CPU [%]
//	float64 memutil		# Memory utilization [%]
//	uint8 gps_fix		# GPS fix: 0=Invalid, 1=GPS, 2=DGPS,4=RTK fixed, 5=RTK float
//	uint8 gps_sats		# Number of satellites in view

}
void updateTelemetry(const ros::TimerEvent&) {
	theta += 0.1;
	fakebat = sin(theta);

	gps_common::GPSFix gps; // -> GPS_MAP
	air2ground.position.x += fakebat;
	air2ground.position.y += fakebat;
	gps.latitude = air2ground.position.x;
	gps.longitude = air2ground.position.y;

	if (flip) {
		gps.status.status = -1;
	} else {
		gps.status.status = 0;
	}
	flip = !flip;

	asctec_msgs::LLStatus battery; // -> GAUGE

	battery.battery_voltage_1 = (9 * fakebat + 9) * 1000;

	asctec_msgs::IMUCalcData imuCalc; //VARIOMETER and COMPASS
	imuCalc.angle_yaw;
	imuCalc.angle_nick;
	imuCalc.angle_roll;
	imuCalc.dheight;
	imuCalc.mag_heading;

	mav_msgs::Height height; // -> ALTIMETER
	air2ground.position.z += 0.1;
	height.height = air2ground.position.z;
	height.climb = 1.0;

	sensor_msgs::Imu imu; // -> COMPASS and ARTIFICIAL_HORIZON
	imu.orientation.x = air2ground.orientation.x; //quaternion rot, maybe change, alle we need is euler
	imu.orientation.y = air2ground.orientation.y;
	imu.orientation.z = air2ground.orientation.z;
//	imu.orientation.w;

	gps_publisher.publish(gps);
	bat_publisher.publish(battery);
	var_comp_publisher.publish(imuCalc);
	alt_publisher.publish(height);
	comp_arth_publisher.publish(imu);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "GroundTele_node");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string subscribe_topic_id;
	std::string publish_topic_id;

	//	Header header
	//	fmVector3 orientation	# Orientation vector (ϕ, θ, ψ)
	//	fmVector3 position	# Position vectos (Pn, Pe, Alt)
	//	float64 airspeed	# Speed through air (Pitot measure) [m/s]
	//	float64 truespeed	# True airspeed (wind compendsated) [m/s]
	//	float64 groundspeed	# Speed over ground [m/s]
	//	float64 battery		# Battery voltage [mV] - ALT: State of Charge [%]
	//	float64 cpuload		# Load on CPU [%]
	//	float64 memutil		# Memory utilization [%]
	//	uint8 gps_fix		# GPS fix: 0=Invalid, 1=GPS, 2=DGPS,4=RTK fixed, 5=RTK float
	//	uint8 gps_sats		# Number of satellites in view
	air2ground.orientation.x = 0;
	air2ground.orientation.y = 0;
	air2ground.orientation.z = 0;
	air2ground.position.x = 55.37412; //lat
	air2ground.position.y = 10.398503; //lon
	air2ground.position.z = 10; //alt
	air2ground.battery = 16.5;

	n.param<std::string>("subscribe_topic_id", subscribe_topic_id,
			"/fmTeleGround/zigbee");

	ros::Subscriber sub = n.subscribe(subscribe_topic_id, 10, xbeeCallback);

	gps_publisher = n.advertise<gps_common::GPSFix>(gpsDataTopic, 1);
	bat_publisher = n.advertise<asctec_msgs::LLStatus>(llStatusTopic, 1);
	var_comp_publisher = n.advertise<asctec_msgs::IMUCalcData>(imuCalcDataTopic,
			1);
	alt_publisher = n.advertise<mav_msgs::Height>(heightTopic, 1);
	comp_arth_publisher = n.advertise<sensor_msgs::Imu>(imuTopic, 1);

	ros::Timer timer1 = nh.createTimer(ros::Duration(0.1), updateTelemetry);

	ros::spin();
	return 0;
}
