#ifndef CCV_BODY_ROTATIONAL_CONTROLLER_H
#define CCV_BODY_ROTATIONAL_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sq2_ccv_roll_pitch_msgs/RollPitch.h>
#include <ccv_dynamixel_msgs/CmdPoseByRadian.h>
#include <math.h>

class CcvBodyRotationalController
{
public:
    CcvBodyRotationalController();

    void process();
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    void cmd_pos_callback(const ccv_dynamixel_msgs::CmdPoseByRadian::ConstPtr& msg);

private:

    double calc_roll(double v, double w);
    double MAX_ROLL_ANGLE_;
    double MAX_PITCH_ANGLE_;
    double MAX_ROLL_VELOCITY_;
    double MAX_PITCH_VELOCITY_;
    double HZ_;
    double V_;
    double W_;
    double ROLL_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher roll_pitch_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber cmd_pos_sub_;

    geometry_msgs::Twist cmd_vel_;
    ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos_;
    sq2_ccv_roll_pitch_msgs::RollPitch roll_pitch_;
};

#endif // CCV_BODY_ROTATIONAL_CONTROLLER_H