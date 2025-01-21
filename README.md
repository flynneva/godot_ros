# godot_ros

This repo is a [GDExtension](https://docs.godotengine.org/en/latest/tutorials/scripting/gdextension/gdextension_cpp_example.html) meant to connect [Robotic Operating System 2 (ROS 2)](https://docs.ros.org/en/rolling/Installation.html) and the [Godot Game Engine](https://docs.godotengine.org/en/stable/about/index.html).

Uses cases for this extension could range from creating a control UI for a
robot to creating a simulator to validate some algorithms.

## Concept

This GDExtension's core concept is hooking into Godots existing [MainLoop](https://docs.godotengine.org/en/latest/classes/class_mainloop.html)
concept and its default `MainLoop` implementation - the [SceneTree](https://docs.godotengine.org/en/latest/classes/class_scenetree.html).

Within the `SceneTree::initialization()` method we can call our typical ROS 2 `rclcpp::init()` and within the `SceneTree::finalize()` method we can call
`rclcpp::shutdown()`. With this approach, every Godot game or application
will be operating within a valid ROS 2 environment.

With the ROS 2 environment up and running, we now need ROS 2 nodes so that
we can create publishers and subscribers. Luckily, Godot also has a node
concept and we can hook directly into it by placing a `rclcpp::Node` shared
pointer as a member variable within a `Godot::Node`. This approach allows almost any Godot node (or any derivative) to also be a ROS 2 node.

The foundation described above lays the ground work - but to actually do
something useful we need to implement conversion functions between Godot
data types and ROS 2 data types.

## Publish (Godot type to ROS 2 messages)

- [x] `Godot::String` => `std_msgs::msg::String`
- [ ] `Godot::Image` => `sensor_msgs::msg::Image`
- [ ] `Godot::Array` => `sensor_msgs::msg::PointCloud`

Additional types can be added.

Feel free to contribute additional type conversions or
create an issue here in this repository with what you
think is missing.

## Subscribe (ROS 2 messages to Godot Type)

- [ ] `Godot::String` => `std_msgs::msg::String`
- [ ] `Godot::Image` => `sensor_msgs::msg::Image`
- [ ] `Godot::Array` => `sensor_msgs::msg::PointCloud`

Additional types can be added.

Feel free to contribute additional type conversions or
create an issue here in this repository with what you
think is missing.

## Building

For now, refer to [the CI script](.github/workflows/build_test.yml)
for the most up-to-date information on how to build this extension.

## Examples

Refer to [the demos](demo/) for examples of GDScript using this
extension.

## Running Godot with the `godot_ros` GDExtension

```shell
source /opt/ros/<distro>/setup.bash
godot
```

Sourcing of the ROS 2 underlay is extremely important, otherwise you will see an error
something like:

```shell
Error: librcutils.so: cannot open shared object file: No such file or directory.
```

### Determining dependencies of godot_ros extension

If for whatever reason you want to fork and extend this package and additional
features are added - additional dependencies may be introduced to the extension
that will need to be declared in the `demo/godot_ros.gdextension` file.

To determine what dependencies are required:

```shell
readelf -d libgodot_ros.linux.template_debug.x86_64.so

Dynamic section at offset 0xced28 contains 31 entries:
  Tag        Type                         Name/Value
 0x0000000000000001 (NEEDED)             Shared library: [librcutils.so]
 0x0000000000000001 (NEEDED)             Shared library: [librclcpp.so]
 0x0000000000000001 (NEEDED)             Shared library: [libstdc++.so.6]
 0x0000000000000001 (NEEDED)             Shared library: [libm.so.6]
 0x0000000000000001 (NEEDED)             Shared library: [libgcc_s.so.1]
 0x0000000000000001 (NEEDED)             Shared library: [libc.so.6]
 0x0000000000000001 (NEEDED)             Shared library: [ld-linux-x86-64.so.2]
 0x000000000000001d (RUNPATH)            Library runpath: [$ORIGIN]
 ...
```

At the top of the list you'll see all the dependency shared libraries that are
needed to run the extension. Keep in mind you'll also need to run the same `readelf`
command on each dependency library in order to get a complete list of all
required dependencies.
