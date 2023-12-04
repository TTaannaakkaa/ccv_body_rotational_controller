#include "ccv_body_rotational_controller/ccv_body_rotational_controller.h"

CcvBodyRotationalController::CcvBodyRotationalController(void) : local_nh_("~")
{
    local_nh_.getParam("MAX_ROLL_ANGLE", MAX_ROLL_ANGLE_);
    local_nh_.getParam("MAX_PITCH_ANGLE", MAX_PITCH_ANGLE_);
    local_nh_.getParam("MAX_ROLL_VELOCITY", MAX_ROLL_VELOCITY_);
    local_nh_.getParam("MAX_PITCH_VELOCITY", MAX_PITCH_VELOCITY_);
    local_nh_.getParam("HZ", HZ_);

    roll_pitch_pub_ = nh_.advertise<sq2_ccv_roll_pitch_msgs::RollPitch>("roll_pitch", 1);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &CcvBodyRotationalController::cmd_vel_callback, this, ros::TransportHints().tcpNoDelay());
    cmd_pos_sub_ = nh_.subscribe("cmd_pos", 1, &CcvBodyRotationalController::cmd_pos_callback, this, ros::TransportHints().tcpNoDelay());
}

void CcvBodyRotationalController::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel_ = *msg;
}

void CcvBodyRotationalController::cmd_pos_callback(const ccv_dynamixel_msgs::CmdPoseByRadian::ConstPtr& msg)
{
    cmd_pos_ = *msg;
}

void CcvBodyRotationalController::process(void)
{
    ros::Rate loop_rate(HZ_);
    while (ros::ok())
    {
        ROS_WARN_STREAM("hogehoge");
        ros::spinOnce();
        loop_rate.sleep();
    }
}

