#include "ccv_body_rotational_controller/ccv_body_rotational_controller.h"

CcvBodyRotationalController::CcvBodyRotationalController(void) : local_nh_("~")
{
    local_nh_.getParam("MAX_ROLL_ANGLE", MAX_ROLL_ANGLE_);
    local_nh_.getParam("MAX_PITCH_ANGLE", MAX_PITCH_ANGLE_);
    local_nh_.getParam("MAX_ROLL_VELOCITY", MAX_ROLL_VELOCITY_);
    local_nh_.getParam("MAX_PITCH_VELOCITY", MAX_PITCH_VELOCITY_);
    local_nh_.getParam("HZ", HZ_);

    ROS_INFO_STREAM("MAX_ROLL_ANGLE: " << MAX_ROLL_ANGLE_);
    ROS_INFO_STREAM("MAX_PITCH_ANGLE: " << MAX_PITCH_ANGLE_);
    ROS_INFO_STREAM("MAX_ROLL_VELOCITY: " << MAX_ROLL_VELOCITY_);
    ROS_INFO_STREAM("MAX_PITCH_VELOCITY: " << MAX_PITCH_VELOCITY_);
    ROS_INFO_STREAM("HZ: " << HZ_);

    roll_pitch_pub_ = nh_.advertise<sq2_ccv_roll_pitch_msgs::RollPitch>("/roll_pitch", 1);
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &CcvBodyRotationalController::cmd_vel_callback, this);
    cmd_pos_sub_ = nh_.subscribe("/cmd_pos", 1, &CcvBodyRotationalController::cmd_pos_callback, this);

}

void CcvBodyRotationalController::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel_ = *msg;
    V_ = cmd_vel_.linear.x;
    W_ = -cmd_vel_.angular.z;
}

void CcvBodyRotationalController::cmd_pos_callback(const ccv_dynamixel_msgs::CmdPoseByRadian::ConstPtr& msg)
{
    cmd_pos_ = *msg;
}

double CcvBodyRotationalController::calc_roll(double v, double w)
{
    return (asin((v*w*0.3/9.81/1.0)/(sqrt(1 + (v*w/9.81)*(v*w/9.81)))) - asin((-v*w/9.81)/(sqrt(1 + (v*w/9.81)*(v*w/9.81)))));
}

void CcvBodyRotationalController::process(void)
{
    ros::Rate loop_rate(HZ_);
    while (ros::ok())
    {
        ROLL_ = CcvBodyRotationalController::calc_roll(V_, W_);
        ROS_WARN_STREAM("ROLL: " << ROLL_ << ", V: " << V_ << ", W: " << W_);
        roll_pitch_.roll = ROLL_;
        roll_pitch_pub_.publish(roll_pitch_);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

