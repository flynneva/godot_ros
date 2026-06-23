#include <godot_cpp/classes/scene_tree.hpp>
#include "godot_ros/scene_tree.hpp"

#include <rclcpp/rclcpp.hpp>

namespace godot
{

void RosSceneTree::_initialize()
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    SceneTree::_initialize();
}

void RosSceneTree::_finalize()
{
    SceneTree::_finalize();
}

void RosSceneTree::_bind_methods() {}

}
