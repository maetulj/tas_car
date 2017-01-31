/************************
 * cmd_vel_to_ackermann.cpp
 *
 * author: Gasper Simonic <gasper.simonic@tume.de>
 *
 * This file includes the transformation from the cmd_vel to ackermann_cmd
 * which is used in the Gazebo simulation.
 */

#include "tas_car_control/cmd_vel_to_ackermann.h"


CmdVelToAckermann::CmdVelToAckermann()
{
    // Subscribe to the "servo" topic. 
    cmd_vel_sub = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &CmdVelToAckermann::cmdVelCallback, this);
    ackermann_cmd_pub = nh_.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd", 1);
}

/**
 * Convert cmd_vel to ackermann_msgs and publish it to ackermann_cmd.
 */
void CmdVelToAckermann::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    double speed = msg->linear.x;
    double w = msg->angular.z;

    // Convert to ackermann steering.
    convertTransRotToSteeringAngle(speed, w, WHEELBASE);

    // Check values.
    if (m_speed > 0 && m_speed > MAX_SPEED)
    {
        m_speed = MAX_SPEED;
    }
    else if (m_speed < 0 && m_speed < -1 * MAX_SPEED)
    {
        m_speed = -MAX_SPEED;
    }

    if (m_steering_angle > 0 && m_steering_angle > MAX_STEERING_ANGLE)
    {
        m_steering_angle = MAX_STEERING_ANGLE;
    }
    else if (m_steering_angle < 0 && m_steering_angle < -1 * MAX_STEERING_ANGLE)
    {
        m_steering_angle = MAX_STEERING_ANGLE;
    }

    // Publish the data.
    ackermann.speed = m_speed;
    ackermann.steering_angle = m_steering_angle;

    ackermann_cmd_pub.publish(ackermann);

    // Refresh the variables.
    m_speed = 0.0;
    m_steering_angle = 0.0;
}

void CmdVelToAckermann::convertTransRotToSteeringAngle(double speed, double w, double wheelbase)
{
    if (speed == 0 || w == 0)
    {
        m_speed = 0.0;
        m_steering_angle = 0.0;

        return;
    }

    double radius = speed / w;

    // Prepare the speed for ackermann_cmd.
    m_speed = speed;
    m_steering_angle = atan(wheelbase / radius);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_to_ackermann");
    CmdVelToAckermann cmd_to_ackermann;

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}