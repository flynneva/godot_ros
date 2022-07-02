extends Spatial

# initialize timeout signal
signal timeout

# calls rclcpp::init(0, nullptr) to make sure ROS is started up
var ros_cmd_listener = CmdListener.new()

var g_timer = null

var linear_x = 0.0
var angular_z = 0.0
# Called when the node enters the scene tree for the first time.
func _ready():

	g_timer = Timer.new()
	add_child(g_timer)
	# connect the talk signal with the timeout callback
	g_timer.connect("timeout", self, "_on_Timer_timeout")
	g_timer.set_wait_time(0.1)
	g_timer.set_one_shot(false)
	g_timer.start()

# called on timeout signal
func _on_Timer_timeout():
	# Spin
	ros_cmd_listener.spin_some()
	var cmd_data_str = ros_cmd_listener.get_cmd()
	if cmd_data_str != "NOCMD":
		var cmd_data_array = cmd_data_str.split("_", true, 1)
		
		var num_size = cmd_data_array.size()
		if num_size == 2:
			linear_x = float(cmd_data_array[0])
			angular_z = float(cmd_data_array[1])
			
		else:
			pass
				
	
func get_cmd_vel_lin_x():
	return  linear_x

func get_cmd_vel_an_z():
	return  angular_z
