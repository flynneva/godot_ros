/* ros.cpp */

#include "ros.h"

String ROS::get_name()
{
  //convert Godot String to Godot CharString to C string
  // p_txt.ascii().get_data());
  return "true";
}

void ROS::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_name"), &ROS::get_name);
}
