#!/usr/bin/env python
import os
import sys
from pathlib import Path
ros_distro = os.environ.get("ROS_DISTRO", "rolling")
cpp_version = "-std=c++17"

ros_dir = "/opt/ros/" + ros_distro

env = Environment()
env.SConscript("godot-cpp/SConstruct", "env")

# godot-cpp statically links libstdc++ on Linux for portability, but that
# means this extension carries its own private copy of std::locale/etc.
# ROS's shared libraries (e.g. libfastrtps) dynamically link the system
# libstdc++, so mixing the two copies corrupts the heap when objects
# allocated by one are destroyed by the other (e.g. std::locale::_Impl
# during rclcpp/FastDDS participant creation). Link dynamically instead so
# this extension shares the same libstdc++ as ROS.
if env["platform"] == "linux":
    env["LINKFLAGS"] = [
        flag for flag in env["LINKFLAGS"] if flag not in ("-static-libgcc", "-static-libstdc++")
    ]

# CacheDir('.cache/scons')

# For reference:
# - CCFLAGS are compilation flags shared between C and C++
# - CFLAGS are for C-specific compilation flags
# - CXXFLAGS are for C++-specific compilation flags
# - CPPFLAGS are for pre-processor flags
# - CPPDEFINES are for pre-processor defines
# - LINKFLAGS are for linking flags

# tweak this if you want to use different folders, or more folders, to store your source code in.
env.Append(CPPPATH=["src/", "include/"])
sources = Glob("src/*.cpp")

def getSubDirs(base_path: str, with_base_path: bool = True):
    if not base_path.endswith("/"):
        base_path += "/"
    sub_dirs = [name for name in os.listdir(base_path) if os.path.isdir(base_path + name)]
    if with_base_path:
        sub_dirs = [f"{base_path}/{name}" for name in sub_dirs]
    return sub_dirs


def getLibNames(base_path: str):
    if not base_path.endswith("/"):
        base_path += "/"
    lib_dirs = [
        Path(name).name
        for name in os.listdir(base_path)
        if os.path.isfile(base_path + name)
        if name.endswith(".so") or name.endswith(".a")
        # libs without the "lib" prefix (e.g. qt_gui_cpp.so) can't be
        # resolved via -lNAME and aren't needed for direct linking here
        if name.startswith("lib")
    ]
    return lib_dirs

ros_includes = getSubDirs(ros_dir + "/include")
ros_lib_path = ros_dir + "/lib"
ros_libs = [
    "rclcpp",
    "rcutils",
    "std_msgs__rosidl_typesupport_cpp",
    "sensor_msgs__rosidl_typesupport_cpp",
]

env.Append(CPPPATH=["include"] + ros_includes)
env.Append(LIBPATH=[ros_lib_path])
# ROS needs c++ version compilier flag
env.Append(CXXFLAGS=[cpp_version, "-fexceptions", "-lpthread"])
env.Append(LIBS=ros_libs)

if env["platform"] == "macos":
    library = env.SharedLibrary(
        "demo/bin/libgodot_ros.{}.{}.framework/libgodot_ros.{}.{}".format(
            env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=sources,
    )
elif env["platform"] == "ios":
    if env["ios_simulator"]:
        library = env.StaticLibrary(
            "demo/bin/libgodot_ros.{}.{}.simulator.a".format(env["platform"], env["target"]),
            source=sources,
        )
    else:
        library = env.StaticLibrary(
            "demo/bin/libgodot_ros.{}.{}.a".format(env["platform"], env["target"]),
            source=sources,
        )
else:
    library = env.SharedLibrary(
        "demo/bin/libgodot_ros{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(library)
