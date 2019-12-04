#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <lcm/lcm-cpp.hpp>
//#include <irp_sen_msgs/encoder.h>
#include <lcmtypes++/irp_core/point3d_t.hpp>
//#include "lcmtypes++/irp_core/point3d_t.hpp"

//#include "lcmdefs/cpp/irp_core/point3d_t.hpp"

class example_t
{
public:
    int64_t    timestamp;
    double     position[3];
    double     orientation[4];
    int32_t    num_ranges;
    std::vector< int16_t > ranges;
    std::string name;
    int8_t     enabled;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;
  lcm::LCM lcm_;
  if(!lcm_.good()) {
     std::cout << "LCM not created well " << std::endl;
  }

  std::string hello = "hello";
  int tt = 1;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  irp_core::point3d_t point_sample;
  point_sample.x = 1;
  point_sample.y = 1;
  point_sample.z = 1;


  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
