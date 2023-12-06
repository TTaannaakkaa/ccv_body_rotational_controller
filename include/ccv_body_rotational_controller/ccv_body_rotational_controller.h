#ifndef CCV_BODY_ROTATIONAL_CONTROLLER_H
#define CCV_BODY_ROTATIONAL_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sq2_ccv_roll_pitch_msgs/RollPitch.h>
#include <ccv_dynamixel_msgs/CmdPoseByRadian.h>
#include <math.h>

class CcvBodyRotationalController
{
public:
    CcvBodyRotationalController();

    void process();

private:

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    double calc_com_vel(double v, double w, double roll);
    double calc_roll(double v, double w);
    double MAX_ROLL_ANGLE_;
    double MAX_PITCH_ANGLE_;
    double H_;
    double L_;
    double HZ_;

    double V_;
    double W_;
    double ROLL_;
    double PITCH_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher roll_pitch_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber odom_sub_;

    nav_msgs::Odometry odom_;
    sq2_ccv_roll_pitch_msgs::RollPitch roll_pitch_;
};

#endif // CCV_BODY_ROTATIONAL_CONTROLLER_H