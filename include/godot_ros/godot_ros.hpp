/* ros.h */

#ifndef GODOT__GODOT_ROS__GODOT_ROS_HPP
#define GODOT__GODOT_ROS__GODOT_ROS_HPP

#include "core/reference.h"

#include "rclcpp/rclcpp.hpp"

class RosNode
: public Reference,
  public rclcpp::Node
{
  GDCLASS(RosNode, Reference);
public:
  RosNode() : rclcpp::Node("godot_ros_node") {}

  // use camel-case of ROS node method to avoid conflict
  String getName();

protected:
  static void _bind_methods();
};

class RosInit : public Reference {
  GDCLASS(RosInit, Reference);
public:
  RosInit() {
    rclcpp::init(0, nullptr);
  }

  ~RosInit() {
    rclcpp::shutdown();
  }

  /// adds a given node to the multithreaded executor
  //  void addNode(Object * node) {
  //    auto rosNodeSharedPtr = getShared(node);
  //    m_executor.add_node(rosNodeSharedPtr);
  //  }

  void spin(Object * node) {
    auto rosNode = cast_to<rclcpp::Node>(node);
    std::shared_ptr<rclcpp::Node> rosNodeSharedPtr(rosNode);
    rclcpp::spin(rosNodeSharedPtr);
  }

protected:
  // run the ROS nodes in separate threads
  // rclcpp::executors::MultiThreadedExecutor m_executor;

  static void _bind_methods();

  // TODO(flynneva): figure out a better way to spin the node)
  /// get shared_ptr from Godot Object node
  //   rclcpp::Node::SharedPtr getShared(Object * node) {
  //     auto rosNode = cast_to<rclcpp::Node>(node);
  //     return std::shared_ptr<rclcpp::Node>(rosNode);
  //   }
};
#endif // GODOT__GODOT_ROS__GODOT_ROS_HPP
