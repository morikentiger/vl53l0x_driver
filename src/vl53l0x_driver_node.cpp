/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */
#include <ros/ros.h>
#include <ros/console.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

#include <stdio.h>
#include <stddef.h>	//#define NULL ...
//#include <linux/i2c-dev.h>
#include <VL53L0X.hpp>
//#include <stdexcept>


VL53L0X sensor;

float MAX_RANGE = 1.3; // 1.2m according docs for default mode

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY


void setup()
{

  sensor.initialize();
  sensor.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vl53l0x_driver");
	ros::NodeHandle n;

  ROS_INFO("Initializing node %s in namespace: %s", ros::this_node::getName().c_str(), ros::this_node::getNamespace().c_str() );

	setup();

	// load parameters from rosparam
	int hz;
	std::string range_frame_id_;

	n.param("frequency", hz, 10);
	n.param<std::string>("frame_id", range_frame_id_, "range_link");

	sensor_msgs::Range range_msg;

	ros::Time current_time = ros::Time::now();

  range_msg.header.stamp = current_time;
  range_msg.header.frame_id = range_frame_id_;
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	// VL53L0X system FOV is 25degrees.
  range_msg.field_of_view = 0.4363323;
	// Seed Recommed measure distance 30mm-1000mm
  range_msg.min_range  = 0;
	// TODO: max depends on mode
  range_msg.max_range = MAX_RANGE;

	//ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("distance", 10);
	ros::Publisher range_pub = n.advertise<sensor_msgs::Range>("range", 1, false);
	ros::Rate loop_rate(hz);

	// float distance_past = 0;

	while (ros::ok())
	{
		//std_msgs::Float64 Distance;

    float distance = sensor.readRangeSingleMillimeters();


		if (distance > 8000){
			ROS_DEBUG("OutOfLenge8191: %f", distance);
			// distance = distance_past*1000;
      distance=MAX_RANGE*1000;
		}

		distance = distance/1000.;
		ROS_DEBUG("readRangeSingleMillimeters:%lf",distance);
		if (sensor.timeoutOccurred()) { ROS_DEBUG(" TIMEOUT_loop"); }

		//Distance.data = distance;
		range_msg.range = distance;
		//distance_past = distance;

		//chatter_pub.publish(Distance);
		range_pub.publish(range_msg);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;

}
