/* ros.cpp */

#include "godot_ros.h"

String RosNode::get_name()
{
  //convert Godot String to Godot CharString to C string
  // p_txt.ascii().get_data());
  return "true";
}

void RosNode::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_name"), &RosNode::get_name);
}
