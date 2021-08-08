# godot_ros

This repo is a [Godot Module](https://docs.godotengine.org/en/stable/development/cpp/binding_to_external_libraries.html#using-the-module) meant to connect [Robotic Operating System 2 (ROS2)](https://docs.ros.org/en/galactic/Installation.html) and the [Godot Game Engine](https://docs.godotengine.org/en/stable/about/index.html).

### Quick Start

Make sure to have both ROS2 and the Godot source installed on your workstation.

Clone this repo into your `godot/modules/` directory.

Modify the `godot_ros/SCsub` file with your ROS2 distro and desired cpp compiler flag.

```
# godot_ros/SCsub file within this repo, lines 5 and 6
ros_distro = "galactic"
cpp_version = "-std=c++17"
```

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
	# calls rclcpp::init(0, nullptr) to make sure ROS is started up
	var ros_init = RosInit.new()

	# creates ROS node
	var ros_node = RosNode.new()
	var node_name = ros_node.get_name()

	print("node_name: ", node_name)
	pass # Replace with function body.
```

### Troubleshooting

Make sure you have sourced your ROS overlay **before** trying to start Godot....otherwise you will see this error:

```
./bin/godot.x11.tools.64: error while loading shared libraries: librclcpp.so: cannot open shared object file: No such file or directory
```

To solve this, just run `source /opt/ros/<ros-distro>/setup.bash` and restart Godot.

### Future work

- support other platforms, should be just configuring the `SCsub` file I think (i.e. Windows)
- add other ROS2 features to `godot_ros`
- generate documentation for this module
- add more demos/examples to documentation
