# Demo

This directory contains a small Godot project that uses the `godot_ros`
GDExtension to publish a ROS 2 string topic from a running Godot scene
(`talker.gd` / `talker.tscn`).

These instructions assume a plain Ubuntu 24.04 terminal (no Distrobox or
other container) with ROS 2 Jazzy installed directly on the host.

## 1. Build the extension

Follow [the main README's "Building" section](../README.md#building) to
install dependencies, install Godot, and build `godot_ros` with `scons`.
That produces `demo/bin/libgodot_ros.linux.template_debug.x86_64.so`, which
the `demo/bin/godot_ros.gdextension` file in this directory points to.

## 2. Run the demo

```bash
source /opt/ros/jazzy/setup.bash
cd demo
godot --headless --import   # only needed once, the first time
godot .
```

The running scene's `talker.gd` script starts an `rclcpp::Node` named
`my_ros_node` and publishes an incrementing counter as a
`std_msgs/msg/String` on `/string_topic` once per second.

## 3. Verify it's working

In a second Ubuntu terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /string_topic
```

You should see the published string messages with an incrementing count.

## Troubleshooting

### `error while loading shared libraries: libasound.so.2: cannot open shared object file`

Godot needs a handful of desktop runtime libraries (audio, X11, GL) that a
minimal Ubuntu install (e.g. a bare container/distrobox with only ROS and
build tools installed) won't have. Install them with:

```bash
sudo apt-get install -y libasound2t64 libgl1 libx11-6 libxcursor1 \
  libxinerama1 libxrandr2 libxi6 libxext6 libxrender1 libxss1 \
  libdbus-1-3 libudev1 fontconfig
```

