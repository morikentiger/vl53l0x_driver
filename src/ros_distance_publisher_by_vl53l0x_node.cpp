/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <stdio.h>
#include <stddef.h>	//#define NULL ...
#include <linux/i2c-dev.h>
#include <vl53l0x_driver/VL53L0X.h>


VL53L0X sensor;


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
  sensor.init();
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
	setup();

	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("distance", 10);

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		std_msgs::Float64 Distance;

		float distance = sensor.readRangeSingleMillimeters();
		distance = distance/1000;
		printf("readRangeSingleMillimeters:%lf\n",distance);
		if (sensor.timeoutOccurred()) { printf(" TIMEOUT_loop\n"); }
		
		Distance.data = distance;

		chatter_pub.publish(Distance);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;

}
