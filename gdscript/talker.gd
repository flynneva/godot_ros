extends Node2D

signal timeout

# calls rclcpp::init(0, nullptr) to make sure ROS is started up
var ros_talker = Talker.new()

var g_timer = null

var count = 1

# Called when the node enters the scene tree for the first time.
func _ready():
	# spin nodes that were added to executors
	# not really need for talker demo but will be needed later for subscribing
	# to ROS2 topics
	ros_talker.spin_some()
	
	g_timer = Timer.new()
	add_child(g_timer)
	# connect the talk signal with the talk callback
	g_timer.connect("timeout", self, "_on_Timer_timeout")
	g_timer.set_wait_time(1.0)
	g_timer.set_one_shot(false)
	g_timer.start()
	pass # Replace with function body.

func _on_Timer_timeout():
  print("godot is talking: " + String(count))
  ros_talker.talk(count)
  count += 1