#include "ccv_body_rotational_controller/ccv_body_rotational_controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ccv_body_rotational_controller_node");
    CcvBodyRotationalController ccv_body_rotational_controller;
    ccv_body_rotational_controller.process();
    return 0;
}