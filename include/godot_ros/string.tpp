/// @brief The RosNode implementations for Godot String -> std_msgs::msg::String
#ifndef GODOT_ROS__STRING_HPP
#define GODOT_ROS__STRING_HPP
#include "godot_ros/node.hpp"

#include "std_msgs/msg/string.hpp"

#include <iostream>


namespace godot
{

using GodotStringType = String;
using RosStringType = std_msgs::msg::String;

template <>
inline void RosNode::create_publisher<GodotStringType>(
    const String & topic_name,
    const int & qos)
{
	assert_rclcpp_node_initialized();

    // Cast the Godot type to a C++ type
    std::string topic_name_as_string = topic_name.utf8().get_data();
    // Create the publisher
	auto new_pub = m_node->create_publisher<RosStringType>(
        topic_name_as_string,
        qos
    );
    // Add the new publisher to the map, using the topic name as the key
    // NOTE: this adds the assumption that there is only one publisher to
    // each topic running in Godot...maybe not so nice :/
    m_publishers.emplace(topic_name, std::move(new_pub));
}

template<>
inline void RosNode::publish<GodotStringType>(
    const String & topic_name,
    const GodotStringType & data)
{
    if (m_publishers.find(topic_name) != m_publishers.end()) {
        // get the publisher for this topic
        auto publisher_variant = m_publishers[topic_name];
        auto publisher = std::get<rclcpp::Publisher<RosStringType>::SharedPtr>(publisher_variant);
        // borrow a loaned message from the publisher
        auto loaned_msg = publisher->borrow_loaned_message();
        // fill in message with data from Godot
        std::string data_as_string = data.utf8().get_data();
        auto & msg = loaned_msg.get();
        msg.data = data_as_string;
        // publish data
        publisher->publish(std::move(loaned_msg));
    }
}

}  // namespace godot

#endif  // GODOT_ROS__STRING_HPP
