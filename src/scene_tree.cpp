#include <godot_cpp/classes/scene_tree.hpp>
#include "godot_ros/scene_tree.hpp"

#include <rclcpp/rclcpp.hpp>

namespace godot
{

void RosSceneTree::_initialize()
{
    rclcpp::init(0, nullptr);
}

void RosSceneTree::_finalize()
{
    // Shutdown the ROS 2 context
    rclcpp::shutdown();
}

void RosSceneTree::_bind_methods() {}

}