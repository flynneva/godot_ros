/// @brief The RosNode implementations for Godot Image -> sensor_msgs::msg::Image
#ifndef GODOT_ROS__IMAGE_HPP
#define GODOT_ROS__IMAGE_HPP
#include "godot_ros/node.hpp"

#include <godot_cpp/classes/image.hpp>

#include "sensor_msgs/msg/image.hpp"

#include <iostream>


namespace godot
{

using GodotImageType = Ref<Image>;
using RosImageType = sensor_msgs::msg::Image;

template <>
inline void RosNode::create_publisher<GodotImageType>(
    const String & topic_name,
    const int & qos)
{
	assert_rclcpp_node_initialized();

    // Cast the Godot type to a C++ type
    std::string topic_name_as_string = topic_name.utf8().get_data();
    // Create the publisher
	auto new_pub = m_node->create_publisher<RosImageType>(
        topic_name_as_string,
        qos
    );
    // Add the new publisher to the map, using the topic name as the key
    // NOTE: this adds the assumption that there is only one publisher to
    // each topic running in Godot...maybe not so nice :/
    m_publishers.emplace(topic_name, std::move(new_pub));
}

template<>
inline void RosNode::publish<GodotImageType>(
    const String & topic_name,
    const GodotImageType & data)
{
    if (m_publishers.find(topic_name) != m_publishers.end()) {
        // get the publisher for this topic
        auto publisher_variant = m_publishers[topic_name];
        auto publisher = std::get<rclcpp::Publisher<RosImageType>::SharedPtr>(publisher_variant);
        // borrow a loaned message from the publisher
        auto loaned_msg = publisher->borrow_loaned_message();
        // fill in message with data from Godot
        auto & msg = loaned_msg.get();
        msg.width = data->get_height();
        msg.height = data->get_width();

        // TODO: support other image formats
        msg.encoding = "rgb8";
        msg.is_bigendian = false;
        msg.step = data->get_data().size() / msg.height;
        msg.data.resize(data->get_data().size());
        std::memcpy(&msg.data[0], data->get_data().ptrw(), data->get_data().size());

        // publish data
        publisher->publish(std::move(loaned_msg));
    }
}

}  // namespace godot

#endif  // GODOT_ROS__IMAGE_HPP
