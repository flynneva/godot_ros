/* ros.cpp */

#include "godot_ros/godot_ros.hpp"


String RosNode::getName()
{
  return String(this->get_name());
}

void RosNode::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_name"), &RosNode::getName);
}

void RosInit::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("talk"), &RosInit::talk);
  ClassDB::bind_method(D_METHOD("spin_some"), &RosInit::spin_some);
}
