#include <iostream>

#include "nav_position.h"

using namespace std;

#define DEBUG 1

Nav_position::Nav_position(void)
    :init(false), num_waypoint(40)
{

}

Nav_position::~Nav_position(void)
{

}

void
Nav_position::trajectoryHandler(const visualization_msgs::MarkerArray::ConstPtr& trajectory)
{
    if(init) return;

    cout << "Hello trajectory handler." << endl;
    for(auto iter=trajectory->markers.begin(); iter != trajectory->markers.end(); ++ iter) {
        if(iter->ns.compare("Trajectory 0") == 0) {

            if(iter->points.size() > 10) {
                for(auto waypoint: iter->points) {
                    Point pnt;
                    pnt.x = waypoint.x;
                    pnt.y = waypoint.y;
                    pnt.z = waypoint.z;
                    global_waypoint.push_back(pnt);
                }

                init = true;
            }

        }
    }
}

void
Nav_position::ros_init (ros::NodeHandle &n)
{
    // sensor msg subscription
    
    navpub = n.advertise<nav_msgs::Odometry>("nav_position", 10);
    odompub = n.advertise<nav_msgs::Odometry>("/odom", 10);

    trajectory_sub = n.subscribe("trajectory_node_list", 10, &Nav_position::trajectoryHandler, this);

    //LCM subscribe
    lcm_.subscribe("POS_T", &Nav_position::PosTCallback, this);
    //while(0 == lcm_.handle());
}

void 
Nav_position::PosTCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::pos_t* msg){
    nav_msgs::Odometry odom_data;
    odom_data.header.stamp = ros::Time::now();
    odom_data.header.frame_id = std::string("odom");
    odom_data.child_frame_id = std::string("base_link");

    odom_data.pose.pose.position.x = msg->x;
    odom_data.pose.pose.position.y = msg->y;
    odom_data.pose.pose.position.z = 0.0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(-msg->h);
    odom_data.pose.pose.orientation = odom_quat;

    //set the velocity
    odom_data.twist.twist.linear.x = 0;
    odom_data.twist.twist.linear.y = 0;
    odom_data.twist.twist.angular.z = 0;
    
    odompub.publish(odom_data);
}


void
Nav_position::runOnce (void)
{
    try{
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
    }

    _state[0] = transform.getOrigin().getX();
    _state[1] = transform.getOrigin().getY(),
    _state[2] = transform.getOrigin().getZ(),

    tf::poseTFToMsg(transform, position.pose.pose);
//    tf::pointMsgToTF(transform.inverse(), global_waypoint);

    if( init ) {
        Eigen::Isometry3d Tbw_iso3d;
        tf::transformTFToEigen(transform.inverse(), Tbw_iso3d);
        Eigen::Matrix4d Tbw = Tbw_iso3d.matrix();
//        cout << Tbw.matrix() << endl << endl;

        float min_dist = 1000000.0;
        int min_idx = -1;
        cout <<"Total waypoint: " <<global_waypoint.size() << endl;
        for(int i=0; i<global_waypoint.size(); ++i) {
            Point& a = global_waypoint[i];
            geometry_msgs::Point& b = position.pose.pose.position;
            float dist = sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));//+(a.z-b.z)*(a.z-b.z));

            if(dist < min_dist) {
                min_dist = dist;
                min_idx = i;
            }
        }

        if( (min_idx != -1) && (min_idx+num_waypoint < global_waypoint.size()) ) {
            cout << "min idx is " << min_idx << endl;
            PointCloud pc_waypoint;

            float waypoint_dist_threshold = 0.5;
            Point prev_waypoint;
                
            prev_waypoint.x = global_waypoint[min_idx].x;
            prev_waypoint.y = global_waypoint[min_idx].y;
            prev_waypoint.z = 0.0;
            prev_waypoint.intensity = 0.0f;

            for(int i = min_idx; i < global_waypoint.size(); ++i) {

                Point p_waypoint;
                p_waypoint.x = global_waypoint[i].x;
                p_waypoint.y = global_waypoint[i].y;
                p_waypoint.z = 0.0;
                p_waypoint.intensity = 0.0f;

                float distance = sqrt((p_waypoint.x-prev_waypoint.x)*(p_waypoint.x-prev_waypoint.x) + (p_waypoint.y-prev_waypoint.y)*(p_waypoint.y-prev_waypoint.y));
                if(distance > waypoint_dist_threshold){
                    prev_waypoint = p_waypoint;
                    pc_waypoint.push_back(p_waypoint);
                    if(pc_waypoint.size() > num_waypoint) break;
                }

            }

            pcl::transformPointCloud(pc_waypoint, pc_waypoint, Tbw);

            eurecar::lpc_t *waypoint = new eurecar::lpc_t();
            waypoint->n = num_waypoint;

            for(auto iter = pc_waypoint.begin(); iter != pc_waypoint.end(); ++iter) {

                waypoint->x.push_back(iter->x); // Real waypoint
                waypoint->y.push_back(iter->y); // Real waypoint
                waypoint->z.push_back(iter->z); // dummy
                waypoint->intensity.push_back(0); // dummy

            }

            lcm_.publish("Waypoint", waypoint);
            delete waypoint;
        }
    }


    eurecar::pos_t *current_pose = new eurecar::pos_t();
    current_pose->x = position.pose.pose.position.x;
    current_pose->y = position.pose.pose.position.y;
    lcm_.publish("CurrentPose", current_pose);
    delete current_pose;

    position.header.frame_id = std::string("map");

    // Publish
    navpub.publish(position);

    ros::spinOnce ();
}

/*double*
Odometry::getState (void)
{
    return _state;
}*/

