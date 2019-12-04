#pragma once

#include <iostream>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include "geometry_msgs/Point.h"

#include "nav_msgs/Odometry.h"
#include "visualization_msgs/MarkerArray.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes++/eurecar/lpc_t.hpp>
#include <lcmtypes++/eurecar/pos_t.hpp>

using namespace std;

class Nav_position
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
    typedef pcl::PointXYZI Point;

  	Nav_position (void);
  	~Nav_position (void);

    void trajectoryHandler(const visualization_msgs::MarkerArray::ConstPtr& trajectory);
    void ros_init (ros::NodeHandle &n);
  	void runOnce (void);
//    double* getState (void);
    void PosTCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::pos_t* msg);

    // lcm
    lcm::LCM lcm_;
  private:
    bool _initialized;

    // subscriber and callback function

    // publisher
    ros::Publisher navpub;
    ros::Publisher odompub;
    ros::Subscriber trajectory_sub;
    nav_msgs::Odometry position;

    // state
    double _state[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::StampedTransform base_link_transform;

    bool init;
    PointCloud global_waypoint;
    int num_waypoint;

    //update_enc_cnt (int left, int right);

};
