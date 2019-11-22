#include <waypoint_controller.h>

WaypointManager::WaypointManager(ros::NodeHandle &nh,
                                 const std::string &cmd_vel_topic,
                                 const float wp_tol) :
    _nh(nh),
    _wp_tol(wp_tol),
    _lat_ctl(controller::create_controller(nh, "lateral_position")),
    _f_ctl(controller::create_controller(nh, "lateral_position")),
    _f_ctl_reconf_server(ros::NodeHandle("~/longitudal_pos")),
    _lat_ctl_reconf_server(ros::NodeHandle("~/lateral_pos"))
{
  _vel_pub = nh.advertise<geometry_msgs::TwistStamped>(cmd_vel_topic, 1);

  _f_ctl_reconf_server.setCallback(std::bind(&controller::reconf_callback,
                    "longitudal_pos", std::ref(_f_ctl), std::placeholders::_1));

  _lat_ctl_reconf_server.setCallback(std::bind(&controller::reconf_callback,
                    "lateral_pos", std::ref(_lat_ctl), std::placeholders::_1));
}

void WaypointManager::add(const geometry_msgs::PoseStamped::ConstPtr &wp){
  _wp_queue.push_back(*wp);
}

void WaypointManager::run(){
  if(_wp_queue.size() <= 0){
    return;
  }

  const auto &curr = _wp_queue.front();

  geometry_msgs::PoseStamped transformed_wp;
  try{
    listener.transformPose("base_link", curr, transformed_wp);
  }catch (tf::TransformException e) {
    ROS_WARN("Could not get transform to base_link, skipping %s",
             e.what());
  }

  const auto t = transformed_wp.header.stamp.toSec();

  const float diffx = transformed_wp.pose.position.x;
  const float diffy = transformed_wp.pose.position.y;

  if (diffx*diffx + diffy*diffy > _wp_tol*_wp_tol){
    _wp_queue.pop_front();
  } else {
    _f_ctl.measure(diffx, t);
    _lat_ctl.measure(diffy, t);

    geometry_msgs::TwistStamped controls;

    controls.header = transformed_wp.header;
    controls.twist.linear.x = _f_ctl.control();
    controls.twist.linear.y = _lat_ctl.control();

    _vel_pub.publish(controls);
  }
}

int main(){
  ros::NodeHandle nh("~");

  float tol = 1;
  std::string cmd_vel_topic;
  std::string wp_add_topic;
  nh.getParam("acceptance_radius", tol);
  nh.getParam("cmd_vel_topic", cmd_vel_topic);
  WaypointManager wp_handler(nh, cmd_vel_topic, tol);

//ros::Subscriber add_point = nh.subscribe<geometry_msgs::PoseStamped>
  //                       (wp_add_topic, 1, &WaypointManager::add, &wp_handler);

  ros::spin();
}
