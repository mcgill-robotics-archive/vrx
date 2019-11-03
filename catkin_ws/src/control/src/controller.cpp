#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <control/PIDParamsConfig.h>

#include <std_msgs/Float32.h>

#include <functional>
#include <memory>
#include <tuple>

#include <pid.h>

typedef std::tuple<ros::Publisher,
    ros::Publisher,
    ros::Publisher,
    ros::Publisher>
    ThrustPublishers;

typedef pid::Controller<float> Controller;

void setpoint_callback(Controller& yr,
    Controller& lv,
    Controller& v,
    const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  yr.set_target(msg->twist.angular.z);
  lv.set_target(msg->twist.linear.y);
  v.set_target(msg->twist.linear.x);
}

void odom_callback(Controller& yr,
    Controller& lv,
    Controller& v,
    const ThrustPublishers& thruster_pubs,
    const nav_msgs::Odometry::ConstPtr& msg)
{
  auto time = msg->header.stamp.toSec();
  auto& twist = msg->twist.twist;
  yr.measure(twist.angular.z, time);
  lv.measure(twist.linear.y, time);
  v.measure(twist.linear.x, time);

  auto cmd_yaw = yr.control();
  auto cmd_lat = lv.control();
  auto cmd_v = v.control();

  auto sum = cmd_yaw + cmd_lat + cmd_v;

  float w1, w2, w3 = 0.;

  if (sum > 0) {
    w1 = cmd_yaw / sum;
    w2 = cmd_lat / sum;
    w3 = cmd_v / sum;
  }

  // mixing the controls
  std_msgs::Float32 thrust_front_left, thrust_front_right;
  std_msgs::Float32 thrust_back_left, thrust_back_right;

  thrust_front_left.data = -w1 * cmd_yaw + w2 * cmd_lat + w3 * cmd_v;
  thrust_front_right.data = w1 * cmd_yaw - w2 * cmd_lat + w3 * cmd_v;
  thrust_back_left.data = -w1 * cmd_yaw - w2 * cmd_lat + w3 * cmd_v;
  thrust_back_right.data = w1 * cmd_yaw + w2 * cmd_lat + w3 * cmd_v;

  std::get<0>(thruster_pubs).publish(thrust_front_left);
  std::get<1>(thruster_pubs).publish(thrust_front_right);
  std::get<2>(thruster_pubs).publish(thrust_back_left);
  std::get<3>(thruster_pubs).publish(thrust_back_right);
}

Controller create_controller(const ros::NodeHandle& nh,
    const std::string& controller_type,
    bool angular = false)
{
  float kp, ki, kd;
  float windup, min_effort, max_effort;

  // TODO: Add error handling if these don't exist..
  nh.getParam(controller_type + "/Kp", kp);
  nh.getParam(controller_type + "/Ki", ki);
  nh.getParam(controller_type + "/Kd", kd);

  nh.getParam(controller_type + "/max_windup", windup);
  nh.getParam(controller_type + "/min_effort", min_effort);
  nh.getParam(controller_type + "/max_effort", max_effort);
  ROS_INFO("Parameters loaded for controller [%s]: Kp=%f Ki=%f Kd=%f min_u=%f max_u=%f", controller_type.c_str(), kp, ki, kd, min_effort, max_effort);

  return pid::make_controller(kp, ki, kd,
      windup, min_effort, max_effort, angular);
}

void reconf_callback(const char* name, Controller &ctl, control::PIDParamsConfig &cfg)
{
  ROS_INFO("Parameters reconfigured for controller [%s]: Kp=%f Ki=%f Kd=%f", name, cfg.Kp, cfg.Ki, cfg.Kd);
  ctl.reconfigure(cfg.Kp, cfg.Ki, cfg.Kd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rate_controller");

  ros::NodeHandle nh("~");

  // Get topics
  std::string odom_topic;
  std::string setpoint_topic;
  std::string front_left_thrust_topic;
  std::string front_right_thrust_topic;
  std::string back_left_thrust_topic;
  std::string back_right_thrust_topic;

  // TODO: Add error handling if these don't exist.. where's std::option??
  nh.getParam("odom_topic", odom_topic);
  nh.getParam("setpoint_topic", setpoint_topic);

  nh.getParam("front_left_thrust_topic", front_left_thrust_topic);
  nh.getParam("front_right_thrust_topic", front_right_thrust_topic);
  nh.getParam("back_left_thrust_topic", back_left_thrust_topic);
  nh.getParam("back_right_thrust_topic", back_right_thrust_topic);

  Controller yaw_rate_controller = create_controller(nh, "yaw_rate", true);
  Controller lateral_vel_controller = create_controller(nh, "lateral_vel", false);
  Controller forward_vel_controller = create_controller(nh, "forward_vel", false);

  ros::Publisher thrust_front_left = nh.advertise<std_msgs::Float32>(front_left_thrust_topic, 1);
  ros::Publisher thrust_front_right = nh.advertise<std_msgs::Float32>(front_right_thrust_topic, 1);
  ros::Publisher thrust_back_left = nh.advertise<std_msgs::Float32>(back_left_thrust_topic, 1);
  ros::Publisher thrust_back_right = nh.advertise<std_msgs::Float32>(back_right_thrust_topic, 1);

  ThrustPublishers thrust_pubs = std::make_tuple(thrust_front_left,
      thrust_front_right,
      thrust_back_left,
      thrust_back_right);

  auto _odom_callback = std::bind(&odom_callback,
      std::ref(yaw_rate_controller),
      std::ref(lateral_vel_controller),
      std::ref(forward_vel_controller),
      std::cref(thrust_pubs),
      std::placeholders::_1);

  auto _setpoint_callback = std::bind(&setpoint_callback,
      std::ref(yaw_rate_controller),
      std::ref(lateral_vel_controller),
      std::ref(forward_vel_controller),
      std::placeholders::_1);

  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1, _odom_callback);
  ros::Subscriber setpoint_sub = nh.subscribe<geometry_msgs::TwistStamped>(setpoint_topic, 1, _setpoint_callback);

  auto yaw_ctl_reconf_cb = std::bind(&reconf_callback,
                                     "yaw_rate",
                                     std::ref(yaw_rate_controller),
                                     std::placeholders::_1);
  auto lat_ctl_reconf_cb = std::bind(&reconf_callback,
                                     "lateral_vel",
                                     std::ref(lateral_vel_controller),
                                     std::placeholders::_1);
  auto f_ctl_reconf_cb = std::bind(&reconf_callback,
                              "forward_vel",
                              std::ref(forward_vel_controller),
                              std::placeholders::_1);
  dynamic_reconfigure::Server<control::PIDParamsConfig> yaw_ctl_reconf_server(ros::NodeHandle("~/yaw_rate"));
  yaw_ctl_reconf_server.setCallback(yaw_ctl_reconf_cb);

  dynamic_reconfigure::Server<control::PIDParamsConfig> lateral_ctl_reconf_server(ros::NodeHandle("~/lateral_vel"));
  lateral_ctl_reconf_server.setCallback(lat_ctl_reconf_cb);

  dynamic_reconfigure::Server<control::PIDParamsConfig> forward_ctl_reconf_server(ros::NodeHandle("~/forward_vel"));
  forward_ctl_reconf_server.setCallback(f_ctl_reconf_cb);

  ros::spin(); // who needs Multithreading
  return 0;
}
