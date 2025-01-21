#!/usr/bin/env python
import os
import sys
from pathlib import Path
ros_distro = os.environ.get("ROS_DISTRO", "rolling")
cpp_version = "-std=c++17"

ros_dir = "/opt/ros/" + ros_distro

env = Environment()
env.SConscript("godot-cpp/SConstruct", "env")

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
    ]
    return lib_dirs

ros_includes = getSubDirs(ros_dir + "/include")
ros_lib_path = ros_dir + "/lib"
ros_libs = getLibNames(ros_lib_path)

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
