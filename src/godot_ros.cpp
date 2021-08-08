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
  // ClassDB::bind_method(D_METHOD("add_node", "node") &RosInit::addNode);
  ClassDB::bind_method(D_METHOD("spin", "node"), &RosInit::spin);
}
