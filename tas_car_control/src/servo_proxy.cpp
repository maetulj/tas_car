/************************
 * servo_proxy.cpp
 *
 * author: Gasper Simonic <gasper.simonic@tume.de>
 *
 * This file includes the implementation of the ServoProxy class
 * for transforming servo commands to Gazebo commands.
 * It also includes the main function of this node.
 */

#include "tas_car_control/servo_proxy.h"


ServoProxy::ServoProxy()
{
	// Subscribe to the "servo" topic. 
	servo_sub = nh_.subscribe<geometry_msgs::Vector3>("servo", 1000, &ServoProxy::servoCallback, this);
	servo_pub = nh_.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd", 1);
}


void ServoProxy::servoCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
	if (msg->x == 1500 && msg->y == 1500)
	{
		// Servo is not moving.
		speed = 0.0;
		steering_angle = 0.0;

		// Publish speed and steering angle.
		// Publish the data.
		ackermann.speed = speed;
		ackermann.steering_angle = steering_angle;

		servo_pub.publish(ackermann);

		return;
	}
	
	// Determine if going forward or backwards.
	x = msg->x - 1500;
	y = msg->y - 1500;

	if (x == 0) 
	{
		speed = 0;
	}
	else if (x > 0)
	{
		// Forward. Determine how much is the nunchuck forward.
		speed = x / SCALE_FAKTOR_THROTTLE_FORWARD;

		if (speed > MAX_SPEED)
		{
			speed = MAX_SPEED;
		}
	}
	else if (x < 0)
	{
		// Backwards.
		speed = x / SCALE_FAKTOR_THROTTLE_BACKWARDS;

		if (speed < -1 * MAX_SPEED)
		{
			speed = -1 * MAX_SPEED;
		}
	}

	// Determine the steering angle..
	steering_angle = y / SCALE_FAKTOR_STEERING;

	if (steering_angle > MAX_STEERING_ANGLE)
	{
		steering_angle = MAX_STEERING_ANGLE;
	}

	// Publish the data.
	ackermann.speed = speed;
	ackermann.steering_angle = steering_angle;

	servo_pub.publish(ackermann);

	// Refresh the variables.
	speed = 0.0;
	steering_angle = 0.0;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "servo_proxy");
    ServoProxy proxy;

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
    	ros::spinOnce();
    	loop_rate.sleep();
    }

	return 0;
}