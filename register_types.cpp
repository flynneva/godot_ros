/* register_types.cpp */

#include "register_types.h"

#include "core/class_db.h"
#include "ros.h"

void register_ros_types() {
    ClassDB::register_class<ROS>();
}

void unregister_ros_types() {
    // Nothing to do here in this example.
}
