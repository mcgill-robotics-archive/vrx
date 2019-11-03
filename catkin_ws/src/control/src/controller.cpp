#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

#include <tuple>
#include <boost/function.hpp>

#include <pid.h>

typedef std::tuple<ros::Publisher,
    ros::Publisher,
    ros::Publisher,
    ros::Publisher>
    MotorPublishers;

typedef pid::Controller<float> Controller;

void setpoint_callback(Controller& yr,
    Controller& lv,
    Controller& v,
    const geometry_msgs::TwistStamped& msg)
{
  yr.set_target(msg.twist.angular.z);
  lv.set_target(msg.twist.linear.y);
  v.set_target(msg.twist.linear.x);
}

void odom_callback(Controller& yr,
    Controller& lv,
    Controller& v,
    const MotorPublishers& motors,
    const nav_msgs::Odometry& msg)
{
  auto time = msg.header.stamp.toSec();
  auto& twist = msg.twist.twist;
  yr.measure(twist.angular.z, time);
  lv.measure(twist.linear.y, time);
  v.measure(twist.linear.x, time);

  auto cmd_yaw = yr.control();
  auto cmd_lat = lv.control();
  auto cmd_v = v.control();

  auto sum = cmd_yaw + cmd_lat + cmd_v;

  auto w1 = cmd_yaw / sum;
  auto w2 = cmd_lat / sum;
  auto w3 = cmd_v / sum;

  // mixing the controls
  auto motor_front_left = -w1 * cmd_yaw + w2 * cmd_lat + w3 * cmd_v;
  auto motor_front_right = w1 * cmd_yaw - w2 * cmd_lat + w3 * cmd_v;
  auto motor_back_left = -w1 * cmd_yaw - w2 * cmd_lat + w3 * cmd_v;
  auto motor_back_right = w1 * cmd_yaw + w2 * cmd_lat + w3 * cmd_v;

  std::get<0>(motors).publish(motor_front_left);
  std::get<1>(motors).publish(motor_front_right);
  std::get<2>(motors).publish(motor_back_left);
  std::get<3>(motors).publish(motor_back_right);
}

Controller create_controller(const ros::NodeHandle& nh,
    const std::string& controller_type,
    bool angular = false)
{
  float kp, ki, kd;
  float windup, min_effort, max_effort;

  // TODO: Add error handling if these don't exist..
  nh.getParam("~" + controller_type + "/Kp", kp);
  nh.getParam("~" + controller_type + "/Ki", ki);
  nh.getParam("~" + controller_type + "/Kd", kd);

  nh.getParam("~" + controller_type + "/max_windup", kp);
  nh.getParam("~" + controller_type + "/min_effort", kd);
  nh.getParam("~" + controller_type + "/max_effort", ki);

  return pid::make_controller(kp, ki, kd,
                                windup, min_effort, max_effort, angular);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rate_controller");

  ros::NodeHandle nh;

  // Get topics
  std::string odom_topic;
  std::string setpoint_topic;
  std::string front_left_motor_topic;
  std::string front_right_motor_topic;
  std::string back_left_motor_topic;
  std::string back_right_motor_topic;

  // TODO: Add error handling if these don't exist.. where's std::option??
  nh.getParam("~odom_topic", odom_topic);
  nh.getParam("setpoint_topic", setpoint_topic);

  nh.getParam("~front_left_motor_topic", front_left_motor_topic);
  nh.getParam("~front_right_motor_topic", front_right_motor_topic);
  nh.getParam("~back_left_motor_topic", back_left_motor_topic);
  nh.getParam("~back_right_motor_topic", back_right_motor_topic);

  Controller yaw_rate_controller = create_controller(nh, "yaw_rate", true);
  Controller lateral_vel_controller = create_controller(nh, "lateral_vel", false);
  Controller forward_vel_controller = create_controller(nh, "forward_vel", false);

  ros::Publisher motor_front_left = nh.advertise<std_msgs::Float32>(front_right_motor_topic, 1);
  ros::Publisher motor_front_right = nh.advertise<std_msgs::Float32>(front_right_motor_topic, 1);
  ros::Publisher motor_back_left = nh.advertise<std_msgs::Float32>(back_left_motor_topic, 1);
  ros::Publisher motor_back_right = nh.advertise<std_msgs::Float32>(back_right_motor_topic, 1);

  MotorPublishers motors = std::make_tuple(motor_front_left,
                                           motor_front_right,
                                           motor_back_left,
                                           motor_back_right);

  auto _odom_callback = std::bind(odom_callback,
                                  yaw_rate_controller,
                                  lateral_vel_controller,
                                  forward_vel_controller,
                                  motors);

  auto _setpoint_callback = std::bind(setpoint_callback,
                                      yaw_rate_controller,
                                      lateral_vel_controller,
                                      forward_vel_controller);

  ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1, _odom_callback);
  ros::Subscriber setpoint_sub = nh.subscribe(setpoint_topic, 1, _setpoint_callback);

  ros::spin(); // who needs Multithreading
  return 0;
}
