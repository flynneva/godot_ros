#ifndef GODOT_ROS__SCENE_TREE_HPP
#define GODOT_ROS__SCENE_TREE_HPP

#include <godot_cpp/classes/scene_tree.hpp>

namespace godot {

class RosSceneTree : public SceneTree {
	GDCLASS(RosSceneTree, SceneTree)

protected:
	static void _bind_methods();

public:
    void _initialize() override;
    void _finalize() override;
};

}

#endif  // GODOT_ROS__SCENE_TREE_HPP