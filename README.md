# godot_ros

This repo is a [Godot Module](https://docs.godotengine.org/en/stable/development/cpp/binding_to_external_libraries.html#using-the-module) meant to connect [Robotic Operating System 2 (ROS2)](https://docs.ros.org/en/galactic/Installation.html) and the [Godot Game Engine](https://docs.godotengine.org/en/stable/about/index.html).

### Quick Start

Make sure to have both ROS2 and the Godot source installed on your workstation.

Clone this repo into your `godot/modules/` directory.

Modify the `godot_ros/SCsub` file with your ROS2 distro and desired cpp compiler flag.

Compile Godot:

```
# make sure you are in the Godot source root directory
cd godot/
scons -j8 platform=x11  # specific for linux/ubuntu
```

Once compiled, you should be able to start Godot:

```
cd godot/
./bin/godot.x11.tools.64
```

And be able to use the `RosNode` Godot object in a GDScript:

```
# Called when the node enters the scene tree for the first time.
func _ready():
	var ros_node = RosNode.new()
	var node_name = ros_node.get_name()
	print("node_name: ", node_name)
```


### Future work

- add other ROS2 features to `godot_ros`
- generate documentation for this module
- add more demos/examples to documentation