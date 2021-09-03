/* register_types.cpp */

#include "register_types.h"
#include "core/class_db.h"
#include "godot_ros/demos/talker.hpp"
#include "godot_ros/demos/view_port.hpp"

void register_godot_ros_types() {
    ClassDB::register_class<Talker>();
    ClassDB::register_class<ViewPort>();

}

void unregister_godot_ros_types() {
    // Nothing to do here in this example.
}
