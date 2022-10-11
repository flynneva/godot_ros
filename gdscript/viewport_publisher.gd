extends Node3D

# initialize timeout signal
signal timeout
var g_timer = null
var count = 1

# initialize ROS viewport node
var ros_viewport = ViewPort.new()

# Called when the node enters the scene tree for the first time.
func _ready():
	# start up ROS node
	ros_viewport.spin_some()
	

func _on_timer_timeout():
	# print("godot viewport publishing" + String.chr(count))

	# Wait until the frame has finished before getting the texture.
	await RenderingServer.frame_post_draw
	# Retrieve the captured image.
	var img = get_viewport().get_texture().get_image()
	img.convert(Image.FORMAT_RGB8)
	# publish image to ROS
	ros_viewport.pubImage(img)
	count += 1
