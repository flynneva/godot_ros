/* ros.h */

#ifndef GODOT__GODOT_ROS__DEMOS__TALKER_HPP
#define GODOT__GODOT_ROS__DEMOS__TALKER_HPP

#include "core/reference.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public Reference {
  GDCLASS(Talker, Reference);
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
