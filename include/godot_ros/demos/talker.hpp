// Copyright 2021 Evan Flynn
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
#ifndef GODOT__GODOT_ROS__DEMOS__TALKER_HPP
#define GODOT__GODOT_ROS__DEMOS__TALKER_HPP

#include "core/object/ref_counted.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public RefCounted {
  GDCLASS(Talker, RefCounted);
public:
  Talker() {
    rclcpp::init(0, nullptr);
    
    m_node = std::make_shared<rclcpp::Node>("godot_talker_node");

    rclcpp::QoS qos(rclcpp::KeepLast(7));
    m_pub = m_node->create_publisher<std_msgs::msg::String>("talker", qos);
  }

  ~Talker() {
    rclcpp::shutdown();
  }

  inline void spin_some() {
    rclcpp::spin_some(m_node);
  }

  // publish message
  inline void talk(const int & count) {
    m_msg = std::make_unique<std_msgs::msg::String>();
    m_msg->data = "Hello from Godot: " + std::to_string(count);
    RCLCPP_INFO(m_node->get_logger(), "Publishing: '%s'", m_msg->data.c_str());

    m_pub->publish(std::move(m_msg));
  }

protected:
  static void _bind_methods();

  // replace rclcpp::Node with your custom node
  std::shared_ptr<rclcpp::Node> m_node;

  // publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub;

  // message to publish
  std::unique_ptr<std_msgs::msg::String> m_msg;
};
#endif // GODOT__GODOT_ROS__DEMOS__TALKER_HPP
