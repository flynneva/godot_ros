/* godot_ros.cpp */

#include "godot_ros/godot_ros.hpp"

void Talker::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("talk", "count"), &Talker::talk);
  ClassDB::bind_method(D_METHOD("spin_some"), &Talker::spin_some);
}
