#include <waypoint_controller.h>

WaypointManager::WaypointManager(ros::NodeHandle &nh,
                                 const std::string &cmd_vel_topic,
                                 const float wp_tol) :
    _nh(nh),
    _wp_tol(wp_tol),
    _lat_ctl(controller::create_controller(nh, "lateral_pos")),
    _f_ctl(controller::create_controller(nh, "longitudal_pos")),
    _yaw_ctl(controller::create_controller(nh, "yaw_pos", true)),
    _f_ctl_reconf_server(ros::NodeHandle("~/longitudal_pos")),
    _lat_ctl_reconf_server(ros::NodeHandle("~/lateral_pos")),
    _yaw_ctl_reconf_server(ros::NodeHandle("~/yaw_pos"))
{
  _vel_pub = nh.advertise<geometry_msgs::TwistStamped>(cmd_vel_topic, 1);

  _f_ctl_reconf_server.setCallback(std::bind(&controller::reconf_callback,
                    "longitudal_pos", std::ref(_f_ctl), std::placeholders::_1));

  _lat_ctl_reconf_server.setCallback(std::bind(&controller::reconf_callback,
                    "lateral_pos", std::ref(_lat_ctl), std::placeholders::_1));

   _yaw_ctl_reconf_server.setCallback(std::bind(&controller::reconf_callback,
                    "yaw_pos", std::ref(_yaw_ctl), std::placeholders::_1));


}

void WaypointManager::add(const geometry_msgs::PoseStamped::ConstPtr &wp){
  ROS_INFO("PUSHED POINT");
  _wp_queue.push_back(*wp);
}

void WaypointManager::clear(const std_msgs::Bool::ConstPtr &msg){
  ROS_INFO("CLEARED WAYPOINTS");
  _wp_queue.clear();
}

void WaypointManager::run(){
  if(_wp_queue.size() <= 0){
    return;
  }

  const auto &curr = _wp_queue.front();

  geometry_msgs::PoseStamped transformed_wp;
  try{
    listener.transformPose("wamv/base_link", curr, transformed_wp);
  }catch (tf::TransformException e) {
    ROS_WARN_THROTTLE(1, "Could not get transform to base_link, skipping %s",
             e.what());
  }

  const auto t = transformed_wp.header.stamp.toSec();

  const float diffx = transformed_wp.pose.position.x;
  const float diffy = transformed_wp.pose.position.y;

  double roll, pitch, yaw;
  tf::Quaternion q(transformed_wp.pose.orientation.x,
                   transformed_wp.pose.orientation.y,
                   transformed_wp.pose.orientation.z,
                   transformed_wp.pose.orientation.w);

  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  const float diffyaw = (float)yaw;

  _f_ctl.measure(-diffx, t);
  _lat_ctl.measure(-diffy, t);
  _yaw_ctl.measure(-diffyaw, t);

  geometry_msgs::TwistStamped controls;
  controls.header = transformed_wp.header;

  if (diffx*diffx + diffy*diffy < _wp_tol*_wp_tol){
    controls.twist.angular.z = _yaw_ctl.control();

    if (std::abs(diffyaw) < _wp_tol){
      ROS_INFO_THROTTLE(5, "REACHED WAYPOINT");
      if(_wp_queue.size() > 1) // only pop if there is more waypoints, otherwise hold..
        _wp_queue.pop_front();
    }
  } else {
    controls.twist.linear.x = _f_ctl.control();
    controls.twist.linear.y = _lat_ctl.control();
  }
  _vel_pub.publish(controls);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_controller");
  ros::NodeHandle nh("~");

  float tol = 1;
  std::string cmd_vel_topic;
  std::string wp_add_topic;
  std::string wp_clear_topic;

  nh.getParam("acceptance_radius", tol);
  nh.getParam("cmd_vel_topic", cmd_vel_topic);
  nh.getParam("wp_add_topic", wp_add_topic);
  nh.getParam("wp_clear_topic", wp_clear_topic);

  WaypointManager wp_handler(nh, cmd_vel_topic, tol);

  ros::Subscriber add_point = nh.subscribe<geometry_msgs::PoseStamped>
                        (wp_add_topic, 1, &WaypointManager::add, &wp_handler);

  ros::Subscriber clear = nh.subscribe<std_msgs::Bool>
                    (wp_clear_topic,  1, &WaypointManager::clear, &wp_handler);

  while(ros::ok()){
    wp_handler.run();
    ros::spinOnce();
  }
}
