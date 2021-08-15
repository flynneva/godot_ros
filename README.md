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

A simple GDScript talker demo would look something like this:

``` GDSCript
extends Node2D

signal timeout

# calls rclcpp::init(0, nullptr) to make sure ROS is started up
var ros_talker = Talker.new()

var g_timer = null

var count = 1

# Called when the node enters the scene tree for the first time.
func _ready():
	# spin nodes that were added to executors
	# not really need for talker demo but will be needed later for subscribing
	# to ROS2 topics
	ros_talker.spin_some()
	
	g_timer = Timer.new()
	add_child(g_timer)
	# connect the talk signal with the talk callback
	g_timer.connect("timeout", self, "_on_Timer_timeout")
	g_timer.set_wait_time(1.0)
	g_timer.set_one_shot(false)
	g_timer.start()
	pass # Replace with function body.

func _on_Timer_timeout():
  print("godot is talking: " + String(count))
  ros_talker.talk(count)
  count += 1
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
