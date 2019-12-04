#include <iostream>

#include <ros/ros.h>

#include "nav_position.h"

using namespace std;

int main(int argc, char** argv)
{
//    Slam slam;

    // ros node initialization
    ros::init(argc, argv, "Nav_position");
    ros::NodeHandle nh;

    // main loop
    Nav_position nav;
    nav.ros_init (nh);

    while(nh.ok() && nav.lcm_.handle() == 0){
        nav.runOnce ();
        ros::Duration(0.1).sleep();
    }

    return 0;
}
