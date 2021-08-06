/* ros.h */

#ifndef GODOT_GODOT_ROS_H
#define GODOT_GODOT_ROS_H

#include "core/reference.h"

#include "rclcpp/rclcpp.hpp"

class RosNode : public Reference, public rclcpp::Node {
  GDCLASS(RosNode, Reference);
public:
  String get_name();

  RosNode() : rclcpp::Node("godot_ros_node") {}

protected:
  static void _bind_methods();

};

#endif // GODOT_GODOT_ROS_H
