extends Node3D

var ros_node = RosNode.new()

var count = 0
var elapsed_time = 0

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	ros_node.init_rclcpp_node("image_node")  # Initialize the rclcpp::Node
	ros_node.create_image_publisher("/image_topic", 10)
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	elapsed_time += delta
	if elapsed_time >= 1:
		elapsed_time = 0
		var img = get_viewport().get_texture().get_image()
		img.convert(Image.FORMAT_RGB8)
		ros_node.publish_image("/image_topic", img)
		count += 1
		print("Published image: " + str(count))
	pass
