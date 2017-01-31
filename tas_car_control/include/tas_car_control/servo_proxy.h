/************************
 * servo_proxy.h
 *
 * author: Gasper Simonic <gasper.simonic@tume.de>
 *
 * This file defines the ServoProxy class for transforming the servo commands
 * to commands for movement in the Gazebo simulation.
 */

#ifndef SERVO_PROXY_H
#define SERVO_PROXY_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <ackermann_msgs/AckermannDrive.h>

#define PI                      		3.14159265
#define CAR_LENGTH              		0.355

#define MAX_SPEED 						0.2
#define MAX_STEERING_ANGLE				0.7

#define SCALE_FAKTOR_THROTTLE_FORWARD  	200
#define SCALE_FAKTOR_THROTTLE_BACKWARDS 300
#define SCALE_FAKTOR_STEERING 			500


class ServoProxy
{
public:
	ServoProxy();

private:
	ros::NodeHandle nh_;

	// x and y servo commands.
	double x, y;

	// Speed.
	double speed;

	// Steering.
	double steering_angle;

	/* flags for the control mode and brake */
    std_msgs::Int16 emergencyBrake;
    std_msgs::Int16 controlMode;

    // Messages.
    geometry_msgs::Vector3 servo;
    ackermann_msgs::AckermannDrive ackermann;

    /* Subscriber to the servo topic. */
    ros::Subscriber servo_sub;

    /* Callback for the servo topic. */
    void servoCallback(const geometry_msgs::Vector3::ConstPtr &msg);

    /* Publisher to the ackermann_cmd topic. */
    ros::Publisher servo_pub;
};



#endif /* SERVO_PROXY_H */