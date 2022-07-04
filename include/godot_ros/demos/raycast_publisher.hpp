// Copyright 2022 Miguel Angel Rodriguez
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
#ifndef GODOT__GODOT_ROS__DEMOS__RayCastPublisher_HPP
#define GODOT__GODOT_ROS__DEMOS__RayCastPublisher_HPP

#include "core/reference.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/point32.hpp"
// https://docs.ros2.org/galactic/api/sensor_msgs/msg/PointCloud.html

class RayCastPublisher : public Reference {
  GDCLASS(RayCastPublisher, Reference);
public:
  RayCastPublisher() {
    rclcpp::init(0, nullptr);
    
    m_node = std::make_shared<rclcpp::Node>("godot_RayCastPublisher_node");

    rclcpp::QoS qos(rclcpp::KeepLast(7));
    m_pub = m_node->create_publisher<sensor_msgs::msg::PointCloud>("ray_cast", qos);
  }

  ~RayCastPublisher() {
    rclcpp::shutdown();
  }

  inline void spin_some() {
    rclcpp::spin_some(m_node);
  }

  // publish message
  inline void publish_raycast_msg(const Array ray_cast_array) {
    m_msg = std::make_unique<sensor_msgs::msg::PointCloud>();

    m_msg->header.frame_id = "world";

    int length_array = ray_cast_array.size();

    for (int i = 0; i < length_array; i++) {
      Vector3 new_position = ray_cast_array[i];
      // In Godot X = X Axis, but Y and Z are inverted, so we correct for that
      float x_val = new_position.x;
      float y_val = new_position.z;
      float z_val = new_position.y;
      geometry_msgs::msg::Point32 tl;
      tl.x = x_val;
      tl.y = y_val;
      tl.z = z_val;
      m_msg->points.push_back(tl);
    }


    m_pub->publish(std::move(m_msg));

  }

protected:
  static void _bind_methods();

  // replace rclcpp::Node with your custom node
  std::shared_ptr<rclcpp::Node> m_node;

  // publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr m_pub;

  // message to publish
  std::unique_ptr<sensor_msgs::msg::PointCloud> m_msg;
};
#endif // GODOT__GODOT_ROS__DEMOS__RayCastPublisher_HPP
