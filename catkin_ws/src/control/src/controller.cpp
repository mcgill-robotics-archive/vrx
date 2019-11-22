#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <control/PIDParamsConfig.h>
#include <control/MixerParamsConfig.h>
#include <dynamic_reconfigure/server.h>

#include <std_msgs/Float32.h>

#include <functional>
#include <memory>
#include <tuple>

#include <pid.h>

#include <controller.h>

using namespace controller;



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
    const Mixer_f& mixers,
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

  // mixing the controls
  auto thrust_vals = mixers.mix(cmd_yaw, cmd_lat, cmd_v);

  std_msgs::Float32 thrust_front_left;
  std_msgs::Float32 thrust_front_right;
  std_msgs::Float32 thrust_back_left;
  std_msgs::Float32 thrust_back_right;

  thrust_front_left.data = thrust_vals[0];
  thrust_front_right.data = thrust_vals[1];
  thrust_back_left.data = thrust_vals[2];
  thrust_back_right.data = thrust_vals[3];

  std::get<0>(thruster_pubs).publish(thrust_front_left);
  std::get<1>(thruster_pubs).publish(thrust_front_right);
  std::get<2>(thruster_pubs).publish(thrust_back_left);
  std::get<3>(thruster_pubs).publish(thrust_back_right);
}

void mixer_reconf_callback(Mixer_f& mixer, control::MixerParamsConfig& cfg){
  mixer.set_w1(cfg.front_left_weight);
  mixer.set_w2(cfg.front_right_weight);
  mixer.set_w3(cfg.back_left_weight);
  mixer.set_w4(cfg.back_right_weight);
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

  float w1, w2, w3, w4;
  nh.getParam("front_left_thruster_weight", w1);
  nh.getParam("front_right_thruster_weight", w2);
  nh.getParam("back_left_thruster_weight", w3);
  nh.getParam("back_right_thruster_weight", w4);

  Mixer_f thrust_mixer(w1, w2, w3, w4);

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
      std::cref(thrust_mixer),
      std::cref(thrust_pubs),
      std::placeholders::_1);

  auto _setpoint_callback = std::bind(&setpoint_callback,
      std::ref(yaw_rate_controller),
      std::ref(lateral_vel_controller),
      std::ref(forward_vel_controller),
      std::placeholders::_1);

  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1,
                                                               _odom_callback);
  ros::Subscriber setpoint_sub = nh.subscribe<geometry_msgs::TwistStamped>(
                                        setpoint_topic, 1, _setpoint_callback);

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
  auto mixer_reconf_cb = std::bind(&mixer_reconf_callback,
      std::ref(thrust_mixer),
      std::placeholders::_1);

  dynamic_reconfigure::Server<control::PIDParamsConfig>
                          yaw_ctl_reconf_server(ros::NodeHandle("~/yaw_rate"));
  yaw_ctl_reconf_server.setCallback(yaw_ctl_reconf_cb);

  dynamic_reconfigure::Server<control::PIDParamsConfig>
                   lateral_ctl_reconf_server(ros::NodeHandle("~/lateral_vel"));
  lateral_ctl_reconf_server.setCallback(lat_ctl_reconf_cb);

  dynamic_reconfigure::Server<control::PIDParamsConfig>
                   forward_ctl_reconf_server(ros::NodeHandle("~/forward_vel"));
  forward_ctl_reconf_server.setCallback(f_ctl_reconf_cb);

  dynamic_reconfigure::Server<control::MixerParamsConfig>
                               mixer_reconf_server(ros::NodeHandle("~/mixer"));
  mixer_reconf_server.setCallback(mixer_reconf_cb);

  ros::spin();
  return 0;
}
