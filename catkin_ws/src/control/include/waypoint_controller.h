#ifndef __WAYPOINT_CONTROLLER_H
#define __WAYPOINT_CONTROLLER_H
#include <deque>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <controller.h>

struct WaypointManager {
  public:
    WaypointManager(ros::NodeHandle &nh,
                    const std::string &vel_topic,
                    const float wp_tol);
    ~WaypointManager(){}

    void add(const geometry_msgs::PoseStamped::ConstPtr &wp);
    void clear();

    void run();

  private:

    ros::NodeHandle &_nh;

    dynamic_reconfigure::Server<control::PIDParamsConfig>
                   _f_ctl_reconf_server;


    dynamic_reconfigure::Server<control::PIDParamsConfig>
                   _lat_ctl_reconf_server;

    ros::Publisher _vel_pub;

    const tf::TransformListener listener;

    controller::Controller _lat_ctl;
    controller::Controller _f_ctl;

    std::deque<geometry_msgs::PoseStamped> _wp_queue;
    float _wp_tol;
};

#endif
