/* ros.h */

#ifndef GODOT_ROS_H
#define GODOT_ROS_H

#include "core/reference.h"

#include "rclcpp/rclcpp.hpp"

class ROS : public Reference, public rclcpp::Node {
  GDCLASS(ROS, Reference);
public:
  String get_name();

  ROS() : rclcpp::Node("godot_ros_node") {}

protected:
  static void _bind_methods();

};

#endif // GODOT_ROS_H
