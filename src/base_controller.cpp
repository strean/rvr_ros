/*
   rvr_ros/base_controller.cpp -



*/

#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "Trace.h"
#include "Blackboard.h"
#include "Response.h"

#include "ApiShell.h"
#include "Connection.h"
#include "Drive.h"
#include "IoLed.h"
#include "Power.h"
#include "SensorsDirect.h"
#include "SensorsStream.h"
#include "SystemInfo.h"


double width = 1;
double ticks_per_meter = 100;

double enc_l = 0.0;
double enc_r = 0.0;
double enc_l_old = 0.0;
double end_r_old = 0.0;
double dist_l = 0.0;
double dist_r = 0.0;
double x = 0.0;
double y = 0.0;
double th = 0.0;
double dt = 0.0001;


ros::Time last_time;
geometry_msgs::Quaternion odom;

// Much of the code below is from Rud Merriams's rvrExercise

SerialPort serial_port { "/dev/ttyTHS1", 115200 };
rvr::SendPacket rvr_out { serial_port };
rvr::ReadPacket rvr_in { serial_port };
rvr::Blackboard bb;

mys::TraceStart terr { std::cerr };
mys::TraceStart tout { std::cout };



void cmd_vel_callback(const geometry_msgs::Twist &twist)
{
	ROS_INFO_STREAM("rvr_ros/base_controller: cmd_vel_callback");

	double v_x = twist.linear.x;
	double v_th = twist.angular.z;

	ROS_INFO_STREAM(" Twist.linear.x: " << twist.linear.x << ", Twist.angular.z: " << twist.angular.z);

	double r = 0.0;
	double l = 0.0;

	rvr::Drive drive(bb, rvr_out);

	if ( v_th == 0 ) {
		r = l = v_x;
	} else if ( v_x == 0 ) {
		r = v_th * width / 2.0;
		l = -r;
	} else {
		r = v_x + ( v_th * width / 2.0 );
		l = v_x - ( v_th * width / 2.0 );
	}

	r *= 50;
	l *= 50;

	ROS_INFO_STREAM(" vl = " << l << ", vr = " << r);

	drive.drive(l, r);

}

int main(int argc, char** argv)
{

	ROS_INFO_STREAM("rvr_ros/base_controller: init");

	ros::init(argc, argv, "base_controller");
	ros::NodeHandle n;

	ros::Subscriber subscriber = n.subscribe("cmd_vel", 10, cmd_vel_callback);

	ros::Rate rate(10);

	// set up a thread to read responses
	std::promise<void> end_tasks;
	std::shared_future<void> end_future(end_tasks.get_future());
	rvr::Response resp { rvr_in, bb, end_future };

	auto resp_future = std::async(std::launch::async, std::ref(resp));

	rvr::IoLed led(bb, rvr_out);
	rvr::Power power(bb, rvr_out);
	rvr::Drive drive(bb, rvr_out);
	rvr::SensorsDirect sensors_d(bb, rvr_out);

	power.awake();
	std::this_thread::sleep_for(500ms);

	rvr::SensorsStream sensors_s(bb, rvr_out);

	try {

	// set LEDs to indicate we are running
	uint32_t led32 {
		Led::status_indication_left | Led::status_indication_right |
		Led::headlight_left | Led::headlight_right
	};
	rvr::RvrMsg colors[] {
		{ 0xFF, 0xFF, 0xFF,
		  0xFF, 0xFF, 0xFF,
		  0xFF, 0xFF, 0xFF,
		  0xFF, 0xFF, 0xFF }
		};
	led.allLed(led32, colors[0]);

	drive.resetYaw();

	sensors_d.resetLocatorXY();
	sensors_d.setLocatorFlags(true);
	sensors_s.disableStreaming();
	sensors_s.clearStreaming();
	std::this_thread::sleep_for(50ms);

	sensors_s.streamSpeedVelocityLocator();

	constexpr uint16_t period { 50 };
	sensors_s.enableStreaming(period);

	std::this_thread::sleep_for(std::chrono::milliseconds(period * 2));


	while( ros::ok() )
	{

		double dxy = 0.0;
		double dth = 0.0;
		double v_xy = dxy / dt;
		double v_th = dth / dt;
		ros::Time now = ros::Time::now();

		ros::spinOnce();

		dt = (now - last_time).toSec();
		last_time = now;


		rvr::VelocityData v { sensors_s.velocity().value_or(rvr::VelocityData { }) };
		// terr << code_loc << "Velocity: " << v.x << mys::sp << v.y;



		rate.sleep();
	}

	} catch (std::exception& e) {
		terr << code_loc << e.what();
	}


	sensors_s.disableStreaming();
	sensors_s.clearStreaming();

	power.sleep();

	std::this_thread::sleep_for(1s);
	end_tasks.set_value();
	resp_future.get();

	return 0;
}
