// Copyright 2022 Evan Flynn
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#ifndef GODOT__GODOT_ROS__DEMOS__CmdListener_HPP
#define GODOT__GODOT_ROS__DEMOS__CmdListener_HPP

#include "core/object/ref_counted.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CmdListener : public RefCounted {
  GDCLASS(CmdListener, RefCounted);
public:
  CmdListener() {
    rclcpp::init(0, nullptr);
    
    m_node = std::make_shared<rclcpp::Node>("godot_CmdListener_node");

    rclcpp::QoS qos(rclcpp::KeepLast(7));
    subscription_ = m_node->create_subscription<std_msgs::msg::String>(
      "/godot_cmd_vel", 10, std::bind(&CmdListener::topic_callback, this, std::placeholders::_1));
  }

  ~CmdListener() {
    rclcpp::shutdown();
  }

  inline void spin_some() {
    rclcpp::spin_some(m_node);
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(m_node->get_logger(), "I Heard: '%s'", msg->data.c_str());
    processed_data = msg->data.c_str();
  }

  // publish message
  String get_cmd() {
    return processed_data;
  }

protected:
  static void _bind_methods();

  // replace rclcpp::Node with your custom node
  std::shared_ptr<rclcpp::Node> m_node;

  // publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  String processed_data = "NOCMD";
};
#endif // GODOT__GODOT_ROS__DEMOS__CmdListener_HPP
