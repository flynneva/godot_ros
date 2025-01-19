extends RosNode


var count = 0
var elapsed_time = 0

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	self.init_rclcpp_node("my_ros_node")  # Initialize the rclcpp::Node
	self.create_string_publisher("/string_topic", 10)
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	elapsed_time += delta
	if elapsed_time >= 1:
		elapsed_time = 0
		print("Publish count: " + str(count))
		self.publish_string("/string_topic", str(count))
		count += 1
	pass
