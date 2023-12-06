#include "ccv_body_rotational_controller/ccv_body_rotational_controller.h"

CcvBodyRotationalController::CcvBodyRotationalController(void) : local_nh_("~")
{
    local_nh_.getParam("MAX_ROLL_ANGLE", MAX_ROLL_ANGLE_);
    local_nh_.getParam("MAX_PITCH_ANGLE", MAX_PITCH_ANGLE_);
    local_nh_.getParam("H", H_);
    local_nh_.getParam("L", L_);
    local_nh_.getParam("HZ", HZ_);

    ROS_INFO_STREAM("HZ: " << HZ_);

    roll_pitch_pub_ = nh_.advertise<sq2_ccv_roll_pitch_msgs::RollPitch>("/roll_pitch", 1);
    odom_sub_ = nh_.subscribe("/odom", 1, &CcvBodyRotationalController::odom_callback, this);

}

void CcvBodyRotationalController::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_ = *msg;
    V_ = odom_.twist.twist.linear.x;
    W_ = odom_.twist.twist.angular.z;
    // ROS_ERROR_STREAM("V: " << V_ << ", W: " << W_);
}

double CcvBodyRotationalController::calc_com_vel(double v, double w, double roll)
{
    double theta = roll*M_PI/180.0;
    return (v + L_*w*cos(theta));
}

double CcvBodyRotationalController::calc_roll(double v, double w)
{
    return (asin((v*w*H_)/(9.81*L_*sqrt(1 - pow(v*w*L_/9.81, 2)))) + asin((v*w*L_/9.81)/sqrt(1 - pow(v*w*L_/9.81, 2))));
}

void CcvBodyRotationalController::process(void)
{
    ros::Rate loop_rate(HZ_);
    while (ros::ok())
    {
        double v = CcvBodyRotationalController::calc_com_vel(V_, W_, ROLL_);
        ROLL_ = CcvBodyRotationalController::calc_roll(v, W_);
        ROS_WARN_STREAM("ROLL: " << ROLL_ << ", V: " << v << ", W: " << W_);
        roll_pitch_.roll = ROLL_;
        roll_pitch_pub_.publish(roll_pitch_);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

