/* ros.h */

#ifndef GODOT__GODOT_ROS__DEMOS__VIEW_PORT_HPP
#define GODOT__GODOT_ROS__DEMOS__VIEW_PORT_HPP

#include "core/reference.h"
#include "core/image.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ViewPort : public Reference {
  GDCLASS(ViewPort, Reference);
public:
  ViewPort() {
    rclcpp::init(0, nullptr);
    
    m_node = std::make_shared<rclcpp::Node>("godot_image_node");

    rclcpp::QoS qos(rclcpp::KeepLast(7));
    m_pub = m_node->create_publisher<sensor_msgs::msg::Image>("image", qos);
  }

  ~ViewPort() {
    rclcpp::shutdown();
  }

  inline void spin_some() {
    rclcpp::spin_some(m_node);
  }

  // publish message
  inline void pubImage(Ref<Image> img) {
    m_msg = std::make_unique<sensor_msgs::msg::Image>();
    RCLCPP_INFO(m_node->get_logger(), "Publishing image");

    m_pub->publish(std::move(m_msg));
  }

protected:
  static void _bind_methods();

  // replace rclcpp::Node with your custom node
  std::shared_ptr<rclcpp::Node> m_node;

  // publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub;

  // message to publish
  std::unique_ptr<sensor_msgs::msg::Image> m_msg;
};
#endif // GODOT__GODOT_ROS__DEMOS__VIEW_PORT_HPP
