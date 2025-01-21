#ifndef GODOT_ROS__NODE_HPP
#define GODOT_ROS__NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <godot_cpp/classes/node.hpp>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace godot {

using SupportedPublisherTypes = std::variant<
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr,
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
>;


class RosNode : public Node {
	GDCLASS(RosNode, Node)

private:
  // replace rclcpp::Node with your custom node
  std::shared_ptr<rclcpp::Node> m_node = nullptr;

  /// @brief Map of all publishers of any ROS 2 msg type
  std::map<String, SupportedPublisherTypes> m_publishers = {};

  void assert_rclcpp_node_initialized();

protected:
	static void _bind_methods();

public:
	void init_rclcpp_node(const String & node_name);

	void spin_some();

    template<typename GodotType>
	inline void create_publisher(
		const String & topic_name,
		const int & qos = 10
	) {
		std::cerr << "Unsupported Godot type given to `create_publisher`" << std::endl;
	}

	template<typename GodotType>
	inline void publish(
		const String & topic_name,
		const GodotType & data
	) {
		std::cerr << "Unsuppoted Godot type given to `publish`" << std::endl;
	}
};

}  // namespace godot


/// @brief Include all the template implementations for each data type
#include "godot_ros/string.tpp"
#include "godot_ros/image.tpp"

#endif  // GODOT_ROS__NODE_HPP
