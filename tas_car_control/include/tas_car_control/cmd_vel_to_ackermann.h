/************************
 * cmd_vel_to_ackermann.h
 *
 * author: Gasper Simonic <gasper.simonic@tume.de>
 *
 * This is the definition of the CmdVelToAckermann class which converts
 * cmd_vel to ackermann_cmd.
 */
#ifndef CMD_VEL_TO_ACKERMANN_H
#define CMD_VEL_TO_ACKERMANN_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ackermann_msgs/AckermannDrive.h>

#define WHEELBASE 35

#define MAX_SPEED                      0.2
#define MAX_STEERING_ANGLE              0.7

/**
 * Class that convert cmd_vel to ackermann_cmd
 */
class CmdVelToAckermann
{
public:
    CmdVelToAckermann();

private:
    ros::NodeHandle nh_;

    // Speed.
    double m_speed;

    // Steering.
    double m_steering_angle;

    // Messages.
    geometry_msgs::Twist cmd_vel;
    ackermann_msgs::AckermannDrive ackermann;

    /* Subscriber to the servo topic. */
    ros::Subscriber cmd_vel_sub;

    /* Callback for the servo topic. */
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

    /* Publisher to the ackermann_cmd topic. */
    ros::Publisher ackermann_cmd_pub;

    void convertTransRotToSteeringAngle(double speed, double w, double wheelbase);

};

#endif /* CMD_VEL_TO_ACKERMANN_H */