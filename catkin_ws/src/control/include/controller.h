#ifndef __CONTROLLER_H
#define __CONTROLLER_H
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

namespace controller{

typedef std::tuple<ros::Publisher,
    ros::Publisher,
    ros::Publisher,
    ros::Publisher>
    ThrustPublishers;

typedef pid::Controller<float> Controller;

// 4 Thrust mixer, maybe we can make this more general but pretty useless
// outside of this application
template <typename T>
struct Mixer {
  std::atomic<T> _w1, _w2, _w3, _w4;
  std::atomic<T> scaling;

  Mixer(const T& w1, const T& w2, const T& w3, const T& w4)
      : _w1(w1)
      , _w2(w2)
      , _w3(w3)
      , _w4(w4)
  {
  }

  inline T w1() const { return _w1; };
  inline T w2() const { return _w2; };
  inline T w3() const { return _w3; };
  inline T w4() const { return _w4; };

  inline T set_w1(const T& w)
  {
    _w1 = w;
  }

  inline T set_w2(const T& w)
  {
    _w2 = w;
  }

  inline T set_w3(const T& w)
  {
    _w3 = w;
  }

  inline T set_w4(const T& w)
  {
    _w4 = w;
  }

  std::array<T, 4> mix(const T& yaw_v,
                       const T& lateral_v,
                       const T& forward_v) const
  {
    std::array<T, 4> thrust_data;

    // Only use two thrusters for each control..
    // ten := front_left, two:= front_right etc ...
    float two_four_lateral = std::max(0.f, lateral_v);
    float ten_eight_lateral = -std::min(0.f, lateral_v);

    float ten_two_forward = std::max(0.f, forward_v);
    float eight_four_forward = -std::min(0.f, forward_v);

    float two_eight_yaw = std::max(0.f, yaw_v);
    float ten_four_yaw = -std::min(0.f, yaw_v);

    float fl_lateral_or_yaw = ten_eight_lateral;
    float fr_lateral_or_yaw = two_four_lateral;
    float bl_lateral_or_yaw = ten_eight_lateral;
    float br_lateral_or_yaw = two_four_lateral;

    //if(std::fabs(yaw_v) > std::fabs(lateral_v)){
      fl_lateral_or_yaw += ten_four_yaw;
      fr_lateral_or_yaw += two_eight_yaw;
      bl_lateral_or_yaw += two_eight_yaw;
      br_lateral_or_yaw += ten_four_yaw;
    //}

    thrust_data[0] = w1() * (ten_two_forward + fl_lateral_or_yaw); // thruster @ ten o clock
    thrust_data[1] = w2() * (ten_two_forward + fr_lateral_or_yaw); // thruster @ two o clock
    thrust_data[2] = w3() * (eight_four_forward + bl_lateral_or_yaw);
    thrust_data[3] = w4() * (eight_four_forward + br_lateral_or_yaw);

    return thrust_data;
  }

};

typedef Mixer<float> Mixer_f;

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
  ROS_INFO("Parameters loaded for controller"
           "[%s]: Kp=%f Ki=%f Kd=%f min_u=%f max_u=%f",
          controller_type.c_str(), kp, ki, kd, min_effort, max_effort);

  return pid::make_controller(kp, ki, kd,
      windup, min_effort, max_effort, angular);
}


void reconf_callback(const char* name, Controller& ctl,
                      control::PIDParamsConfig& cfg)
{
  ROS_INFO("Parameters reconfigured for controller [%s]:"
           "Kp=%0.2f Ki=%0.2f Kd=%0.2f min_effort=%0.2f max_effort=%0.2f",
           name, cfg.Kp, cfg.Ki, cfg.Kd, cfg.min_effort, cfg.max_effort);

  ctl.set_Kp(cfg.Kp);
  ctl.set_Kd(cfg.Kd);
  ctl.set_Ki(cfg.Ki);
  ctl.set_min_effort(cfg.min_effort);
  ctl.set_max_effort(cfg.max_effort);
}

} // namespace controller
#endif
